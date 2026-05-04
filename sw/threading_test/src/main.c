/*
 * X-HEEP Threading & Synchronisation Verification Test
 *
 * All thread functions run at PRIO_MID (K_PRIO_PREEMPT(5)).
 * main() thread starts at K_PRIO_PREEMPT(0) (higher priority) and uses k_sleep(K_MSEC(5))
 * to cede the CPU to PRIO_MID threads.
 * k_yield() only yields to same/higher priority.
 * k_futex requires CONFIG_USERSPACE, unavailable on X-HEEP. So it can't be tested.
 * Seq 4 and Seq 7 lower main to PRIO_LOW to exercise contention.
 *
 * Synchronisation primitives:
 *   Seq  1: Thread lifecycle        — k_thread_create / k_thread_join
 *   Seq  2: Suspend / Resume        — k_thread_suspend / k_thread_resume
 *   Seq  3: Binary semaphore        — k_sem_take (blocking) / k_sem_give
 *   Seq  4: Mutex contention        — k_mutex_lock / k_mutex_unlock, hold ~50 ms
 *   Seq  5: Counting semaphore      — give 3×, take 3×, 4th take times out
 *   Seq  6: Mutex reentrancy        — same thread locks twice, unlocks twice
 *   Seq  7: Condition variable      — cond_signal (one waiter) + cond_broadcast (two)
 *   Seq  8: Event object            — k_event_post / k_event_wait / k_event_wait_all
 *   Seq  9: Spinlock                — k_spin_lock / k_spin_unlock around shared counter
 *   Seq 10: k_poll                  — k_poll on semaphore
 *   Seq 11: IRQ lock                — irq_lock / irq_unlock, verify critical section
 *   Seq 12: Atomic operations       — set/get/add/sub/CAS/set_bit/test_bit
 *
 * Data-passing primitives:
 *   Seq 13: FIFO                    — k_fifo_put 3×, k_fifo_get in FIFO order
 *   Seq 14: LIFO                    — k_lifo_put 3×, k_lifo_get in LIFO order
 *   Seq 15: Stack                   — k_stack_push 3×, k_stack_pop in LIFO order
 *   Seq 16: Message queue           — k_msgq_put / k_msgq_peek / k_msgq_get
 *   Seq 17: Mailbox                 — k_mbox_put (main) / k_mbox_get (thread)
 *   Seq 18: Pipe                    — k_pipe_write / k_pipe_read, verify payload
*/


//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>


//! THREADS

//* Priority presets
#define PRIO_MID    K_PRIO_PREEMPT(5)
#define PRIO_LOW    K_PRIO_PREEMPT(10)


//* Stack definitions
#define STACK_SIZE  1024U
K_THREAD_STACK_DEFINE(seq1_stack,   STACK_SIZE);
K_THREAD_STACK_DEFINE(seq2_stack,   STACK_SIZE);
K_THREAD_STACK_DEFINE(seq3_stack,   STACK_SIZE);
K_THREAD_STACK_DEFINE(seq4_stack,   STACK_SIZE);
K_THREAD_STACK_DEFINE(seq5a_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(seq7a_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(seq7b_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(seq7c_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(seq8_stack,   STACK_SIZE);
K_THREAD_STACK_DEFINE(seq9_stack,   STACK_SIZE);
K_THREAD_STACK_DEFINE(seq10_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(seq17_stack,  STACK_SIZE);

//* Thread structs
static struct k_thread seq1_thd, seq2_thd, seq3_thd, seq4_thd;
static struct k_thread seq5a_thd;
static struct k_thread seq7a_thd, seq7b_thd, seq7c_thd;
static struct k_thread seq8_thd;
static struct k_thread seq9_thd;
static struct k_thread seq10_thd;
static struct k_thread seq17_thd;


//* Global Variables (volatiles and semaphores)
// Seq 1
static volatile bool t1_ran;
// Seq 2
static volatile bool t2_started;
static volatile bool t2_resumed;
// Seq 3
K_SEM_DEFINE(bin_sem, 0, 1);
static volatile bool sem3_received;
// Seq 4
K_MUTEX_DEFINE(mtx4);
// Seq 5
K_SEM_DEFINE(count_sem, 0, 3);
// Seq 6
K_MUTEX_DEFINE(mtx6);
// Seq 7
K_MUTEX_DEFINE(cond_mtx);
K_CONDVAR_DEFINE(cond_var);
static volatile int cond_signal_count;
static volatile int cond_broadcast_count;
// Seq 8
K_EVENT_DEFINE(test_event);
// Seq 9
static struct k_spinlock spin9;
static volatile int spin_counter;
// Seq 10
K_SEM_DEFINE(poll_sem, 0, 1);
static volatile bool poll10_ran;
// Seq 17
K_MBOX_DEFINE(test_mbox);


//* Thread Functions

static void seq1_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  t1_ran = true;
}

static void seq2_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  t2_started = true;
  k_thread_suspend(k_current_get());
  t2_resumed = true;
}

static void seq3_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  k_sem_take(&bin_sem, K_FOREVER);
  sem3_received = true;
}

static void seq4_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  k_mutex_lock(&mtx4, K_FOREVER);
  k_sleep(K_MSEC(50));
  k_mutex_unlock(&mtx4);
}

static void seq5_taker_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  int r = k_sem_take(&count_sem, K_MSEC(50));
  /* store result via atomic so main can read it safely */
  atomic_set((atomic_t *)a, r);
}

static void seq7_signal_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  k_mutex_lock(&cond_mtx, K_FOREVER);
  k_condvar_wait(&cond_var, &cond_mtx, K_FOREVER);
  cond_signal_count++;
  k_mutex_unlock(&cond_mtx);
}

static void seq7_bcast_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  k_mutex_lock(&cond_mtx, K_FOREVER);
  k_condvar_wait(&cond_var, &cond_mtx, K_FOREVER);
  cond_broadcast_count++;
  k_mutex_unlock(&cond_mtx);
}

static void seq8_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  k_event_wait_all(&test_event, 0x03, false, K_FOREVER);
  /* signal main that we woke */
  k_event_post(&test_event, BIT(2));
}

static void seq9_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  for (int i = 0; i < 1000; i++) {
    k_spinlock_key_t key = k_spin_lock(&spin9);
    spin_counter++;
    k_spin_unlock(&spin9, key);
  }
}

static void seq10_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
  struct k_poll_event events[1];
  k_poll_event_init(&events[0], K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &poll_sem);
  k_poll(events, 1, K_FOREVER);
  k_sem_take(&poll_sem, K_NO_WAIT);
  poll10_ran = true;
}

static void seq17_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  uint32_t rx_data = 0;
  struct k_mbox_msg rmsg = {
    .size     = sizeof(rx_data),
    .rx_source_thread = K_ANY,
  };

  k_mbox_get(&test_mbox, &rmsg, &rx_data, K_FOREVER);
  atomic_set((atomic_t *)a, (atomic_val_t)rx_data);
}


//! DATA-PASSING

//* Data-Passing structs

// Seq 13 FIFO
struct fifo_item {
  void    *fifo_reserved;  /* required first field */
  uint32_t val;
};

// Seq 14 LIFO
struct lifo_item {
  void    *lifo_reserved;  /* required first field */
  uint32_t val;
};

// Seq 15 Stack
K_STACK_DEFINE(kstack, 4);

// Seq 16 Message Queue
K_MSGQ_DEFINE(test_msgq, sizeof(uint32_t), 4, 4);

// Seq 18: Pipe
K_PIPE_DEFINE(test_pipe, 64, 4);


//! TEST SEQUENCES

static int test_seq_1(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 1: Thread Lifecycle ---\n");

  t1_ran = false;
  k_tid_t tid = k_thread_create(&seq1_thd, seq1_stack, STACK_SIZE,
    seq1_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  int ret = k_thread_join(tid, K_MSEC(500));

  if (ret == 0 && t1_ran) {
    printk("[PASS] thread ran and set flag within 500 ms\n");
    pass++;
  }
  else if (ret != 0) {
    printk("[FAIL] k_thread_join timed out (ret=%d)\n", ret);
    fail++;
  }
  else {
    printk("[FAIL] thread joined but flag not set\n");
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_2(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 2: Thread Suspend / Resume ---\n");

  t2_started = false;
  t2_resumed = false;

  k_tid_t tid = k_thread_create(&seq2_thd, seq2_stack, STACK_SIZE, seq2_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  k_sleep(K_MSEC(5));

  if (t2_started) {
    printk("[PASS] thread ran before suspending\n");
    pass++;
  }
  else {
    printk("[FAIL] t2_started not set before suspend\n");
    fail++;
  }

  if (!t2_resumed) {
    printk("[PASS] thread is suspended (t2_resumed still false)\n");
    pass++;
  }
  else {
    printk("[FAIL] thread ran past suspension point unexpectedly\n");
    fail++;
  }

  k_thread_resume(tid);
  k_thread_join(tid, K_MSEC(500));

  if (t2_resumed) {
    printk("[PASS] thread continued after resume\n");
    pass++;
  }
  else {
    printk("[FAIL] thread did not continue after resume\n");
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_3(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 3: Binary Semaphore ---\n");

  sem3_received = false;

  k_tid_t tid = k_thread_create(&seq3_thd, seq3_stack, STACK_SIZE, seq3_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  k_sleep(K_MSEC(5));

  if (!sem3_received) {
    printk("[PASS] thread blocked on semaphore\n");
    pass++;
  }
  else {
    printk("[FAIL] thread did not block\n");
    fail++;
  }

  k_sem_give(&bin_sem);
  k_thread_join(tid, K_MSEC(500));

  if (sem3_received) {
    printk("[PASS] thread unblocked after sem_give\n");
    pass++;
  }
  else {
    printk("[FAIL] thread did not unblock within 500 ms\n");
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_4(void) {
  /*
    Main is lowered to PRIO_LOW so seq4_fn (PRIO_MID) preempts on creation.
    seq4_fn acquires the mutex and sleeps ~50 ms while holding it.
    Main then blocks on k_mutex_lock -> time elapsed should be ≥ 40 ms.
  */
  int pass = 0, fail = 0;
  printk("\n--- Seq 4: Mutex Contention ---\n");

  k_thread_priority_set(k_current_get(), PRIO_LOW);

  k_thread_create(&seq4_thd, seq4_stack, STACK_SIZE, seq4_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  uint32_t t0 = k_uptime_get_32();
  k_mutex_lock(&mtx4, K_FOREVER);
  uint32_t time_elapsed = k_uptime_get_32() - t0;
  k_mutex_unlock(&mtx4);

  printk("[INFO] mutex wait: %u ms (thread held ~50 ms)\n", time_elapsed);

  if (time_elapsed >= 40U) {
    printk("[PASS] elapsed >= 40 ms\n");
    pass++;
  }
  else {
    printk("[FAIL] elapsed %u ms < 40 ms\n", time_elapsed);
    fail++;
  }
  if (time_elapsed <= 200U) {
    printk("[PASS] elapsed <= 200 ms\n");
    pass++;
  }
  else {
    printk("[FAIL] elapsed %u ms > 200 ms\n", time_elapsed);
    fail++;
  }

  k_thread_join(&seq4_thd, K_MSEC(200));

  /* restore priority for remaining seqs */
  k_thread_priority_set(k_current_get(), K_PRIO_PREEMPT(0));

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_5(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 5: Counting Semaphore ---\n");

  /* Give 3× (limit=3) */
  k_sem_give(&count_sem);
  k_sem_give(&count_sem);
  k_sem_give(&count_sem);

  if (k_sem_count_get(&count_sem) == 3) {
    printk("[PASS] count == 3 after three gives\n");
    pass++;
  } 
  else {
    printk("[FAIL] unexpected count %u\n", k_sem_count_get(&count_sem));
    fail++;
  }

  /* Take 3× — all should succeed immediately */
  int ok = 1;
  for (int i = 0; i < 3; i++) {
    if (k_sem_take(&count_sem, K_NO_WAIT) != 0) {
      ok = 0;
    }
  }
  if (ok) {
    printk("[PASS] all three takes succeeded\n"); pass++;
  }
  else {
    printk("[FAIL] one or more takes failed\n"); fail++;
  }

  /* 4th take — create a helper thread that will time out */
  atomic_t taker_ret;
  atomic_set(&taker_ret, 99);

  k_thread_create(&seq5a_thd, seq5a_stack, STACK_SIZE,
    seq5_taker_fn, &taker_ret, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);
  k_thread_join(&seq5a_thd, K_MSEC(200));

  if (atomic_get(&taker_ret) == -EAGAIN) {
    printk("[PASS] 4th take timed out with -EAGAIN\n");
    pass++;
  }
  else {
    printk("[FAIL] 4th take returned %ld (expected -EAGAIN)\n",
      (long)atomic_get(&taker_ret));
      fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_6(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 6: Mutex Reentrancy ---\n");

  int r1 = k_mutex_lock(&mtx6, K_NO_WAIT);
  int r2 = k_mutex_lock(&mtx6, K_NO_WAIT);

  if (r1 == 0 && r2 == 0) {
    printk("[PASS] same thread locked mutex twice (r1=%d, r2=%d)\n", r1, r2);
    pass++;
  }
  else {
    printk("[FAIL] reentrant lock failed (r1=%d, r2=%d)\n", r1, r2);
    fail++;
  }

  k_mutex_unlock(&mtx6);
  k_mutex_unlock(&mtx6);

  /* After two unlocks the mutex should be fully released */
  int r3 = k_mutex_lock(&mtx6, K_NO_WAIT);
  if (r3 == 0) {
    printk("[PASS] mutex fully released after two unlocks\n");
    pass++;
    k_mutex_unlock(&mtx6);
  }
  else {
    printk("[FAIL] mutex still held after two unlocks (r3=%d)\n", r3); fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_7(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 7: Condition Variable ---\n");

  cond_signal_count    = 0;
  cond_broadcast_count = 0;

  /* Lower main so PRIO_MID waiters can acquire cond_mtx before main signals */
  k_thread_priority_set(k_current_get(), PRIO_LOW);

  /* --- Signal path: one waiter --- */
  k_thread_create(&seq7a_thd, seq7a_stack, STACK_SIZE, seq7_signal_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  k_sleep(K_MSEC(5));  /* let thread reach k_condvar_wait */

  k_mutex_lock(&cond_mtx, K_FOREVER);
  k_condvar_signal(&cond_var);
  k_mutex_unlock(&cond_mtx);

  k_thread_join(&seq7a_thd, K_MSEC(200));

  if (cond_signal_count == 1) {
    printk("[PASS] signal woke exactly one waiter\n");
    pass++;
  }
  else {
    printk("[FAIL] cond_signal_count=%d (expected 1)\n", cond_signal_count);
    fail++;
  }

  /* --- Broadcast path: two waiters --- */
  k_thread_create(&seq7b_thd, seq7b_stack, STACK_SIZE, seq7_bcast_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);
  k_thread_create(&seq7c_thd, seq7c_stack, STACK_SIZE, seq7_bcast_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  k_sleep(K_MSEC(5));  /* let both threads reach k_condvar_wait */

  k_mutex_lock(&cond_mtx, K_FOREVER);
  k_condvar_broadcast(&cond_var);
  k_mutex_unlock(&cond_mtx);

  k_thread_join(&seq7b_thd, K_MSEC(200));
  k_thread_join(&seq7c_thd, K_MSEC(200));

  if (cond_broadcast_count == 2) {
    printk("[PASS] broadcast woke both waiters\n");
    pass++;
  }
  else {
    printk("[FAIL] cond_broadcast_count=%d (expected 2)\n", cond_broadcast_count);
    fail++;
  }

  k_thread_priority_set(k_current_get(), K_PRIO_PREEMPT(0));

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_8(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 8: Event Object ---\n");

  k_event_clear(&test_event, 0xFFFFFFFF);

  /* Thread waits for BIT(0) AND BIT(1) (wait_all), then posts BIT(2) */
  k_thread_create(&seq8_thd, seq8_stack, STACK_SIZE, seq8_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  /* Post only BIT(0) — thread must still be waiting */
  k_event_post(&test_event, BIT(0));
  k_sleep(K_MSEC(5));

  uint32_t ev = k_event_wait(&test_event, BIT(2), false, K_NO_WAIT);
  if (!(ev & BIT(2))) {
    printk("[PASS] thread still waiting after partial post\n");
    pass++;
  }
  else {
    printk("[FAIL] BIT(2) already set — thread should still be waiting\n");
    fail++;
  }

  /* Post BIT(1) — now both bits set, thread wakes and posts BIT(2) */
  k_event_post(&test_event, BIT(1));
  k_thread_join(&seq8_thd, K_MSEC(200));

  ev = k_event_wait(&test_event, BIT(2), false, K_NO_WAIT);
  if (ev & BIT(2)) {
    printk("[PASS] thread woke after BIT(0)|BIT(1) and posted BIT(2)\n");
    pass++;
  }
  else {
    printk("[FAIL] BIT(2) not set — thread did not wake\n");
    fail++;
  }

  /* k_event_wait_all with K_NO_WAIT on complete set */
  ev = k_event_wait_all(&test_event, 0x07, false, K_NO_WAIT);
  if (ev & 0x07) {
    printk("[PASS] k_event_wait_all returned all three bits\n");
    pass++;
  }
  else {
    printk("[FAIL] k_event_wait_all returned 0x%x (expected 0x7)\n", ev);
    fail++;
  }

  /* Clear and verify */
  k_event_clear(&test_event, 0xFFFFFFFF);
  ev = k_event_wait(&test_event, 0x07, false, K_NO_WAIT);
  if (ev == 0) {
    printk("[PASS] k_event_clear cleared all bits\n");
    pass++;
  }
  else {
    printk("[FAIL] bits still set after clear: 0x%x\n", ev);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_9(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 9: Spinlock ---\n");

  spin_counter = 0;

  /* Lower main so thread can preempt and race */
  k_thread_priority_set(k_current_get(), PRIO_LOW);

  k_tid_t tid = k_thread_create(&seq9_thd, seq9_stack, STACK_SIZE, seq9_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  /* Main also increments 1000× under spinlock */
  for (int i = 0; i < 1000; i++) {
    k_spinlock_key_t key = k_spin_lock(&spin9);
    spin_counter++;
    k_spin_unlock(&spin9, key);
  }

  k_thread_join(tid, K_MSEC(500));

  k_thread_priority_set(k_current_get(), K_PRIO_PREEMPT(0));

  if (spin_counter == 2000) {
    printk("[PASS] spin_counter == 2000 (no lost updates)\n");
    pass++;
  }
  else {
    printk("[FAIL] spin_counter == %d (expected 2000)\n", spin_counter);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_10(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 10: k_poll ---\n");

  poll10_ran = false;

  k_tid_t tid = k_thread_create(&seq10_thd, seq10_stack, STACK_SIZE, seq10_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  k_sleep(K_MSEC(5));  /* let thread reach k_poll */

  if (!poll10_ran) {
    printk("[PASS] thread blocked on k_poll\n");
    pass++;
  }
  else {
    printk("[FAIL] thread did not block\n");
    fail++;
  }

  k_sem_give(&poll_sem);  /* signal the semaphore — wakes the poll */
  k_thread_join(tid, K_MSEC(200));

  if (poll10_ran) {
    printk("[PASS] thread unblocked after sem_give and ran\n");
    pass++;
  }
  else {
    printk("[FAIL] thread did not wake within 200 ms\n");
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_11(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 11: IRQ Lock ---\n");

  volatile int counter = 0;
  unsigned int key = irq_lock();
  counter++;
  counter++;
  irq_unlock(key);

  if (counter == 2) {
    printk("[PASS] counter == 2 after irq_lock critical section\n");
    pass++;
  }
  else {
    printk("[FAIL] counter == %d (expected 2)\n", counter);
    fail++;
  }

  /* Re-enable should be idempotent: lock/unlock twice */
  key = irq_lock();
  irq_unlock(key);
  key = irq_lock();
  irq_unlock(key);
  printk("[PASS] double irq_lock/unlock did not hang\n");
  pass++;

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_12(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 12: Atomic Operations ---\n");

  atomic_t a;

  /* set / get */
  atomic_set(&a, 42);
  if (atomic_get(&a) == 42) {
    printk("[PASS] atomic_set/get: 42\n");
    pass++;
  }
  else {
    printk("[FAIL] atomic_get returned %ld\n", (long)atomic_get(&a));
    fail++;
  }

  /* add */
  atomic_add(&a, 8);
  if (atomic_get(&a) == 50) {
    printk("[PASS] atomic_add: 42+8=50\n");
    pass++;
  }
  else {
    printk("[FAIL] after add: %ld\n", (long)atomic_get(&a));
    fail++;
  }

  /* sub */
  atomic_sub(&a, 10);
  if (atomic_get(&a) == 40) {
    printk("[PASS] atomic_sub: 50-10=40\n");
    pass++;
  }
  else {
    printk("[FAIL] after sub: %ld\n", (long)atomic_get(&a)); 
  fail++;
  }

  /* CAS — success */
  bool cas_ok = atomic_cas(&a, 40, 99);
  if (cas_ok && atomic_get(&a) == 99) {
    printk("[PASS] atomic_cas: 40->99 succeeded\n");
    pass++;
  }
  else {
    printk("[FAIL] CAS result: ok=%d val=%ld\n", (int)cas_ok, (long)atomic_get(&a));
    fail++;
  }

  /* CAS — failure (wrong expected value) */
  bool cas_fail = atomic_cas(&a, 0, 1);
  if (!cas_fail && atomic_get(&a) == 99) {
    printk("[PASS] atomic_cas: wrong-expected-value correctly rejected\n");
    pass++;
  }
  else {
    printk("[FAIL] spurious CAS succeeded\n");
    fail++;
  }

  /* set_bit / test_bit */
  atomic_t bits;
  atomic_set(&bits, 0);
  atomic_set_bit(&bits, 3);
  if (atomic_test_bit(&bits, 3)) {
    printk("[PASS] atomic_set_bit/test_bit: bit 3 set\n");
    pass++;
  }
  else {
    printk("[FAIL] bit 3 not set\n");
    fail++;
  }

  /* clear_bit */
  atomic_clear_bit(&bits, 3);
  if (!atomic_test_bit(&bits, 3)) {
    printk("[PASS] atomic_clear_bit: bit 3 cleared\n");
    pass++;
  }
  else {
    printk("[FAIL] bit 3 still set after clear\n");
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_13(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 13: FIFO ---\n");

  struct k_fifo fifo;
  k_fifo_init(&fifo);

  static struct fifo_item items[3] = {
    { .val = 10 }, { .val = 20 }, { .val = 30 }
  };

  k_fifo_put(&fifo, &items[0]);
  k_fifo_put(&fifo, &items[1]);
  k_fifo_put(&fifo, &items[2]);

  struct fifo_item *p;
  uint32_t expected[] = { 10, 20, 30 };
  int ok = 1;

  for (int i = 0; i < 3; i++) {
    p = k_fifo_get(&fifo, K_NO_WAIT);
    if (!p || p->val != expected[i]) {
      printk("[FAIL] fifo[%d]: got %u, expected %u\n", i, p ? p->val : 0xFFFFFFFF, expected[i]);
      ok = 0;
      fail++;
    }
  }
  if (ok) {
    printk("[PASS] FIFO order preserved (10, 20, 30)\n");
    pass++;
  }

  /* 4th get on empty fifo — must return NULL */
  p = k_fifo_get(&fifo, K_NO_WAIT);
  if (!p) {
    printk("[PASS] empty FIFO returns NULL\n");
    pass++;
  }
  else {
    printk("[FAIL] empty FIFO returned non-NULL\n");
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_14(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 14: LIFO ---\n");

  struct k_lifo lifo;
  k_lifo_init(&lifo);

  static struct lifo_item litems[3] = {
    { .val = 10 }, { .val = 20 }, { .val = 30 }
  };

  k_lifo_put(&lifo, &litems[0]);
  k_lifo_put(&lifo, &litems[1]);
  k_lifo_put(&lifo, &litems[2]);

  struct lifo_item *p;
  uint32_t expected[] = { 30, 20, 10 };  /* LIFO: last in, first out */
  int ok = 1;

  for (int i = 0; i < 3; i++) {
    p = k_lifo_get(&lifo, K_NO_WAIT);
    if (!p || p->val != expected[i]) {
      printk("[FAIL] lifo[%d]: got %u, expected %u\n", i, p ? p->val : 0xFFFFFFFF, expected[i]);
      ok = 0;
      fail++;
    }
  }
  if (ok) {
    printk("[PASS] LIFO order correct (30, 20, 10)\n");
    pass++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_15(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 15: k_stack ---\n");

  k_stack_push(&kstack, 100);
  k_stack_push(&kstack, 200);
  k_stack_push(&kstack, 300);

  stack_data_t val;
  uint32_t expected[] = { 300, 200, 100 };
  int ok = 1;

  for (int i = 0; i < 3; i++) {
    int r = k_stack_pop(&kstack, &val, K_NO_WAIT);
    if (r != 0 || (uint32_t)val != expected[i]) {
      printk("[FAIL] stack[%d]: got %u (r=%d), expected %u\n", i, (uint32_t)val, r, expected[i]);
      ok = 0;
      fail++;
    }
  }
  if (ok) {
    printk("[PASS] stack LIFO order correct (300, 200, 100)\n");
    pass++;
  }

  /* Pop from empty — must fail */
  int r = k_stack_pop(&kstack, &val, K_NO_WAIT);
  if (r != 0) {
    printk("[PASS] empty stack pop returns error\n");
    pass++;
  }
  else {
    printk("[FAIL] empty stack pop succeeded unexpectedly\n");
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_16(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 16: Message Queue ---\n");

  uint32_t tx[] = { 0xAA, 0xBB, 0xCC };
  for (int i = 0; i < 3; i++) {
    k_msgq_put(&test_msgq, &tx[i], K_NO_WAIT);
  }

  if (k_msgq_num_used_get(&test_msgq) == 3) {
    printk("[PASS] 3 messages queued\n");
    pass++;
  }
  else {
    printk("[FAIL] queue depth %u (expected 3)\n", k_msgq_num_used_get(&test_msgq));
    fail++;
  }

  /* peek — must not dequeue */
  uint32_t peeked = 0;
  k_msgq_peek(&test_msgq, &peeked);
  if (peeked == 0xAA && k_msgq_num_used_get(&test_msgq) == 3) {
    printk("[PASS] peek returned 0xAA, depth still 3\n");
    pass++;
  }
  else {
    printk("[FAIL] peek=0x%x depth=%u\n", peeked, k_msgq_num_used_get(&test_msgq));
    fail++;
  }

  /* get all three and verify order */
  uint32_t rx;
  int ok = 1;
  for (int i = 0; i < 3; i++) {
    k_msgq_get(&test_msgq, &rx, K_NO_WAIT);
    if (rx != tx[i]) {
      printk("[FAIL] msgq[%d]: got 0x%x, expected 0x%x\n", i, rx, tx[i]);
      ok = 0; fail++;
    }
  }
  if (ok) {
    printk("[PASS] FIFO order preserved (0xAA, 0xBB, 0xCC)\n");
    pass++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_17(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 17: Mailbox ---\n");

  atomic_t rx_val;
  atomic_set(&rx_val, 0);

  /* Thread blocks on k_mbox_get. main() sends one message */
  k_thread_create(&seq17_thd, seq17_stack, STACK_SIZE, seq17_fn, &rx_val, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  k_sleep(K_MSEC(5));  /* let thread reach k_mbox_get */

  uint32_t tx_data = 0x00FABADA;
  struct k_mbox_msg smsg = {
    .size        = sizeof(tx_data),
    .tx_data     = &tx_data,
    .rx_source_thread = K_ANY,
    .tx_target_thread = K_ANY,
  };
  int r = k_mbox_put(&test_mbox, &smsg, K_MSEC(200));

  k_thread_join(&seq17_thd, K_MSEC(500));

  if (r == 0 && (uint32_t)atomic_get(&rx_val) == 0x00FABADA) {
    printk("[PASS] mbox: thread received 0x00FABADA\n");
    pass++;
  }
  else {
    printk("[FAIL] mbox: r=%d rx=0x%lx\n", r, (unsigned long)atomic_get(&rx_val));
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_18(void) {
  int pass = 0, fail = 0;
  printk("\n--- Seq 18: Pipe ---\n");

  uint8_t tx_buf[] = { 0x11, 0x22, 0x33 };
  int written = k_pipe_write(&test_pipe, tx_buf, sizeof(tx_buf), K_NO_WAIT);

  if (written == (int)sizeof(tx_buf)) {
    printk("[PASS] pipe write: %d bytes written\n", written);
    pass++;
  }
  else {
    printk("[FAIL] pipe write: returned %d (expected %zu)\n", written, sizeof(tx_buf));
    fail++;
  }

  uint8_t rx_buf[3] = { 0 };
  int read_bytes = k_pipe_read(&test_pipe, rx_buf, sizeof(rx_buf), K_NO_WAIT);

  if (read_bytes == (int)sizeof(tx_buf)) {
    printk("[PASS] pipe read: %d bytes read\n", read_bytes);
    pass++;
  }
  else {
    printk("[FAIL] pipe read: returned %d (expected %zu)\n", read_bytes, sizeof(tx_buf));
    fail++;
  }

  if (rx_buf[0] == 0x11 && rx_buf[1] == 0x22 && rx_buf[2] == 0x33) {
    printk("[PASS] pipe payload correct (0x11, 0x22, 0x33)\n");
    pass++;
  }
  else {
    printk("[FAIL] pipe payload: 0x%02x 0x%02x 0x%02x\n", rx_buf[0], rx_buf[1], rx_buf[2]);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


//! MAIN

int main(void)
{
  printk("\n========================================\n");
  printk("\tX-HEEP Threading & Sync Test\n");
  printk("\t\tBoard: %s\n", CONFIG_BOARD);
  printk("========================================\n");

  int total_fail = 0;

  total_fail += test_seq_1();
  total_fail += test_seq_2();
  total_fail += test_seq_3();
  total_fail += test_seq_4();
  total_fail += test_seq_5();
  total_fail += test_seq_6();
  total_fail += test_seq_7();
  total_fail += test_seq_8();
  total_fail += test_seq_9();
  total_fail += test_seq_10();
  total_fail += test_seq_11();
  total_fail += test_seq_12();
  total_fail += test_seq_13();
  total_fail += test_seq_14();
  total_fail += test_seq_15();
  total_fail += test_seq_16();
  total_fail += test_seq_17();
  total_fail += test_seq_18();

  printk("\n========================================\n");
  if (total_fail == 0) {
    printk("\tALL TESTS PASSED");
  } else {
    printk("\t%d TEST(S) FAILED", total_fail);
  }
  printk("\n========================================\n");

  return 0;
}
