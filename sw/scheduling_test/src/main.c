/*
 * X-HEEP Scheduler Verification Test
 *
 * Ready-queue policy has to be selected in prj.conf at build-time.
 * Tickless kernel is enabled by default on X-HEEP.
 *
 * Sequence 1: Priority preemption  — PRIO_HIGH thread preempts PRIO_LOW main on create
 * Sequence 2: Round-robin          — two PRIO_LOW threads interleave via k_yield
 * Sequence 3: k_sleep accuracy     — 100 ms sleep measured via k_uptime_get_32 (±100 ms)
 * Sequence 4: Timeout handling     — k_sem_take with 100 ms deadline returns -EAGAIN
 * Sequence 5: Scheduler lock       — k_sched_lock prevents preemption, k_sched_unlock restores it
 * Sequence 6: Time-slice           — two equal-priority threads progress without k_yield
 * Sequence 7: EDF scheduling       — two equal-priority threads, earlier deadline runs first
*/

//! LIBRARIES

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>


//! CONFIG CHECKER MACROS

// TODO: these do not seem to work properly. Debug pending.
BUILD_ASSERT(IS_ENABLED(CONFIG_TICKLESS_KERNEL),    "tickless kernel expected on X-HEEP");
BUILD_ASSERT(IS_ENABLED(CONFIG_SCHED_DEADLINE),     "EDF test requires CONFIG_SCHED_DEADLINE=y");
BUILD_ASSERT(CONFIG_TIMESLICE_SIZE > 0,             "time-slice test requires TIMESLICE_SIZE > 0");


//! THREADS

//* Priority presets
#define PRIO_HIGH   K_PRIO_PREEMPT(1)
#define PRIO_MID    K_PRIO_PREEMPT(5)
#define PRIO_LOW    K_PRIO_PREEMPT(10)

//* Stack definitions
#define STACK_SIZE  1024U
K_THREAD_STACK_DEFINE(seq1_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(rr_a_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(rr_b_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(seq5_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(ts_a_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(ts_b_stack,  STACK_SIZE);
K_THREAD_STACK_DEFINE(edf_a_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(edf_b_stack, STACK_SIZE);

//* Thread Structs
static struct k_thread seq1_thd, rr_a_thd, rr_b_thd;
static struct k_thread seq5_thd, ts_a_thd, ts_b_thd, edf_a_thd, edf_b_thd;


//* Global Variables (volatiles and semaphores)
// Seq 1
static volatile int preempt_order[2];
// Seq 2
static volatile int thread_yield_steps[2];
K_SEM_DEFINE(rr_done, 0, 2);
// Seq 4
K_SEM_DEFINE(timeout_sem, 0, 1);
// Seq 5
static volatile bool sched_lock_preempted;
// Seq 6 — both threads set their bit in ts_flag then spin on the peer's bit;
//         without time-slicing only one thread ever runs and both sem_takes time out
static volatile int ts_flag[2];
K_SEM_DEFINE(ts_done, 0, 2);
// Seq 7
static volatile int edf_order[2];
static volatile int edf_counter;
K_SEM_DEFINE(edf_done, 0, 2);


//* Thread functions

static void seq1_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  preempt_order[0] = 1;
  preempt_order[1] = 1;
}

static void rr_fn(void *arg, void *b, void *c) {
  int id = (int)(intptr_t)arg;
  ARG_UNUSED(b); ARG_UNUSED(c);

  /*
    TODO: might be interesting to check
    max difference between thread_yield_steps[0] and [1] during runtime
  */
  for (int i = 0; i < 5; i++) {
    thread_yield_steps[id]++;
    k_yield();
  }
  k_sem_give(&rr_done);
}

static void sched_lock_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

  sched_lock_preempted = true;
}

static void ts_a_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
  ts_flag[0] = 1;

  while (!ts_flag[1]) {}
  k_sem_give(&ts_done);
}

static void ts_b_fn(void *a, void *b, void *c) {
  ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
  ts_flag[1] = 1;

  while (!ts_flag[0]) {}
  k_sem_give(&ts_done);
}

static void edf_fn(void *arg, void *b, void *c) {
  int id = (int)(intptr_t)arg;
  ARG_UNUSED(b); ARG_UNUSED(c);

  edf_order[id] = ++edf_counter;
  k_sem_give(&edf_done);
}


//! TEST SEQUENCES

static int test_seq_1(void) {
  /*
    The main() is lowered to PRIO_LOW so a PRIO_HIGH thread can preempt it.
    k_thread_create triggers an immediate context switch into seq1_fn.
    seq1_fn sets preempt_order[0]=1 and exits, then main resumes and sets preempt_order[1]=2.
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 1: Priority Preemption ---\n");

  k_thread_priority_set(k_current_get(), PRIO_LOW); // Just as a reminder, Zephyr runs EVERYTHING on a thread

  preempt_order[0] = 0;
  preempt_order[1] = 0;

  k_tid_t tid = k_thread_create(&seq1_thd, seq1_stack, STACK_SIZE, seq1_fn, NULL, NULL, NULL, PRIO_HIGH, 0, K_NO_WAIT);

  preempt_order[1] = 2;

  if (preempt_order[0] == 1 && preempt_order[1] == 2) {
    printk("[PASS] high-prio thread ran before main continued (order: %d then %d)\n",
       preempt_order[0], preempt_order[1]);
    pass++;
  }
  else {
    printk("[FAIL] unexpected order: [0]=%d [1]=%d (expected 1, 2)\n",  preempt_order[0], preempt_order[1]);
    fail++;
  }

  k_thread_join(tid, K_MSEC(100));

  k_thread_priority_set(k_current_get(), PRIO_MID);

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_2(void) {
  /*
    Two threads are created at PRIO_LOW (as main) — no preemption at create.
    Main blocks on rr_done; threads alternate via k_yield until all 5 steps
    are complete, then each gives rr_done once.
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 2: Round-Robin (k_yield) ---\n");

  k_thread_priority_set(k_current_get(), PRIO_LOW);

  printk("[INFO] two PRIO_LOW threads interleave via k_yield, 5 steps each\n");
  thread_yield_steps[0] = 0;
  thread_yield_steps[1] = 0;
  k_thread_create(&rr_a_thd, rr_a_stack, STACK_SIZE, rr_fn, (void *)0, NULL, NULL, PRIO_LOW, 0, K_NO_WAIT);
  k_thread_create(&rr_b_thd, rr_b_stack, STACK_SIZE, rr_fn, (void *)1, NULL, NULL, PRIO_LOW, 0, K_NO_WAIT);

  k_sem_take(&rr_done, K_MSEC(200));
  k_sem_take(&rr_done, K_MSEC(200));

  if (thread_yield_steps[0] == 5) {
    printk("[PASS] thread A completed 5 steps\n");
    pass++;
  }
  else {
    printk("[FAIL] thread A steps=%d (expected 5)\n", thread_yield_steps[0]);
    fail++;
  }

  if (thread_yield_steps[1] == 5) {
    printk("[PASS] thread B completed 5 steps\n");
    pass++;
  }
  else {
    printk("[FAIL] thread B steps=%d (expected 5)\n", thread_yield_steps[1]);
    fail++;
  }

  k_thread_priority_set(k_current_get(), PRIO_MID);

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_3(void) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 3: Kernel-level Sleep Accuracy (tickless kernel) ---\n");

  uint32_t t0 = k_uptime_get_32();
  k_sleep(K_MSEC(100));
  uint32_t time_elapsed = k_uptime_get_32() - t0;

  printk("[INFO] requested 100 ms, elapsed %u ms\n", time_elapsed);

  if (time_elapsed >= 100U) {
    printk("[PASS] elapsed >= 100 ms\n");
    pass++;
  }
  else {
    printk("[FAIL] elapsed %u ms < 100 ms (underrun)\n", time_elapsed);
    fail++;
  }

  if (time_elapsed <= 200U) {
    printk("[PASS] elapsed <= 200 ms (non-excessive overrun)\n");
    pass++;
  }
  else {
    printk("[FAIL] elapsed %u ms > 200 ms (overrun)\n", time_elapsed);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_4(void) {
  int pass = 0, fail = 0;

  printk("\n--- Seq 4: Timeout Handling ---\n");
  printk("[INFO] k_sem_take with 100 ms deadline on an empty semaphore\n");

  uint32_t t0 = k_uptime_get_32();
  int ret = k_sem_take(&timeout_sem, K_MSEC(100));
  uint32_t time_elapsed = k_uptime_get_32() - t0;

  printk("[INFO] ret=%d elapsed=%u ms\n", ret, time_elapsed);

  if (ret == -EAGAIN) {
    printk("[PASS] k_sem_take returned -EAGAIN (timeout as expected)\n");
    pass++;
  }
  else {
    printk("[FAIL] k_sem_take returned %d (expected -EAGAIN)\n", ret);
    fail++;
  }

  if (time_elapsed >= 100U) {
    printk("[PASS] elapsed >= 100 ms (timeout ok)\n");
    pass++;
  }
  else {
    printk("[FAIL] elapsed %u ms < 100 ms (early return)\n", time_elapsed);
    fail++;
  }

  if (time_elapsed <= 200U) {
    printk("[PASS] elapsed <= 200 ms (non-excessive overrun)\n");
    pass++;
  }
  else {
    printk("[FAIL] elapsed %u ms > 200 ms (overrun)\n", time_elapsed);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_5(void) {
  /*
    k_sched_lock() makes the calling thread non-preemptible.
    A PRIO_HIGH thread is created while the lock is held, itis expected that
    will not preempt the running thread until k_sched_unlock().
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 5: Scheduler Lock ---\n");

  sched_lock_preempted = false;

  k_sched_lock();

  k_tid_t tid = k_thread_create(&seq5_thd, seq5_stack, STACK_SIZE, sched_lock_fn, NULL, NULL, NULL, PRIO_HIGH, 0, K_NO_WAIT);

  if (!sched_lock_preempted) {
    printk("[PASS] PRIO_HIGH thread did not preempt while scheduler locked\n");
    pass++;
  }
  else {
    printk("[FAIL] PRIO_HIGH thread ran over scheduler lock\n");
    fail++;
  }

  k_sched_unlock();

  k_thread_join(tid, K_MSEC(100));

  if (sched_lock_preempted) {
    printk("[PASS] PRIO_HIGH thread ran after k_sched_unlock\n");
    pass++;
  }
  else {
    printk("[FAIL] PRIO_HIGH thread did not run after k_sched_unlock\n");
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_6(void) {
  /*
    Two equal-priority threads run without k_yield.
    Each flips up an entry in ts_flag then spins until the opposite entry is set.
    Without time-slicing, only one thread would ever run and both sem_takes would time out.
    Both completing within 2s proves the 1 ms timeslice fires.
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 6: Time-Slice (no k_yield) ---\n");
  printk("[INFO] TIMESLICE_SIZE=%d ms, TIMESLICE_PRIORITY=%d\n", CONFIG_TIMESLICE_SIZE, CONFIG_TIMESLICE_PRIORITY);

  ts_flag[0] = 0;
  ts_flag[1] = 0;

  k_thread_create(&ts_a_thd, ts_a_stack, STACK_SIZE, ts_a_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);
  k_thread_create(&ts_b_thd, ts_b_stack, STACK_SIZE, ts_b_fn, NULL, NULL, NULL, PRIO_MID, 0, K_NO_WAIT);

  int ret_a = k_sem_take(&ts_done, K_MSEC(2000));
  int ret_b = k_sem_take(&ts_done, K_MSEC(2000));

  if (ret_a == 0 && ret_b == 0) {
    printk("[PASS] Both threads progressed without k_yield (time-slice active)\n");
    pass++;
  }
  else {
    printk("[FAIL] The sem_take timed out: ret_a=%d ret_b=%d (only one thread ran — time-slice not firing?)\n",
      ret_a, ret_b);
    fail++;
  }

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


static int test_seq_7(void) {
  /*
    Both threads start after a 10 ms delay to give main time to set their deadlines
    Thread A gets a far deadline (10 000 000 cycles ~ 100ms @ 100MHz).
    Thread B gets a near deadline (100 000 cycles ~ 1ms @ 100MHz).
    The EDF scheduler should pick B first among threads at PRIO_MID.
  */
  int pass = 0, fail = 0;

  printk("\n--- Seq 7: EDF Scheduling ---\n");
  printk("[INFO] thread B deadline < thread A deadline; expect B runs first\n");

  edf_counter = 0;
  edf_order[0] = 0;
  edf_order[1] = 0;

  k_tid_t tid_a = k_thread_create(&edf_a_thd, edf_a_stack, STACK_SIZE, edf_fn, (void *)0, NULL, NULL, PRIO_MID, 0, K_MSEC(10));
  k_tid_t tid_b = k_thread_create(&edf_b_thd, edf_b_stack, STACK_SIZE, edf_fn, (void *)1, NULL, NULL, PRIO_MID, 0, K_MSEC(10));

  k_thread_deadline_set(tid_a, 10000000);
  k_thread_deadline_set(tid_b,   100000);

  k_sem_take(&edf_done, K_MSEC(2000));
  k_sem_take(&edf_done, K_MSEC(2000));

  printk("[INFO] run order: A=%d B=%d (lower = ran first)\n", edf_order[0], edf_order[1]);

  if (edf_order[1] < edf_order[0]) {
    printk("[PASS] B (nearer deadline) ran before A\n");
    pass++;
  }
  else {
    printk("[FAIL] A ran before B — EDF not ordering by deadline\n");
    fail++;
  }

  k_thread_join(tid_a, K_MSEC(100));
  k_thread_join(tid_b, K_MSEC(100));

  printk("\nResult: %d passed, %d failed\n", pass, fail);
  return fail;
}


//! MAIN

int main(void) {
  printk("\n========================================\n");
  printk("\tX-HEEP Scheduling Test\n");
  printk("\t\tBoard: %s\n", CONFIG_BOARD);
  printk("\t\tQueue backend: %s\n",
    IS_ENABLED(CONFIG_SCHED_SCALABLE) ? "SCALABLE (rbtree)" :
    IS_ENABLED(CONFIG_SCHED_MULTIQ)   ? "MULTIQ (array)"    : "SIMPLE (list)");
  printk("========================================\n");

  int total_fail = 0;
  total_fail += test_seq_1();
  total_fail += test_seq_2();
  total_fail += test_seq_3();
  total_fail += test_seq_4();
  total_fail += test_seq_5();
  total_fail += test_seq_6();
  total_fail += test_seq_7();

  printk("\n========================================\n");
  if (total_fail == 0) {
    printk("\tALL TESTS PASSED");
  } else {
    printk("\t%d TEST(S) FAILED", total_fail);
  }
  printk("\n========================================\n");

  return 0;
}
