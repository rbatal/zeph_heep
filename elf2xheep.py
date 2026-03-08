#!/usr/bin/env python3
"""
Convert Zephyr ELF to X-HEEP compatible hex format.

X-HEEP uses $readmemh with byte-oriented format:
@XXXXXXXX           <- byte address (hex)
XX XX XX XX ...     <- space-separated bytes (16 bytes per line)

X-HEEP boots from address 0x180, this script adds a jump
instruction at 0x180 that jumps to address 0x0 (where
Zephyr's __start is located).

Usage:
  python3 elf2xheep.py zephyr.elf output.hex
"""

import sys
import argparse
import struct


def read_elf_segments(elf_path):
    """Read loadable segments from ELF file."""
    segments = []
    entry_point = 0
    
    with open(elf_path, 'rb') as f:
        # Read ELF header
        magic = f.read(4)
        if magic != b'\x7fELF':
            raise ValueError("Not a valid ELF file")
        
        ei_class = struct.unpack('B', f.read(1))[0]
        if ei_class != 1:
            raise ValueError("Only 32-bit ELF supported")
        
        ei_data = struct.unpack('B', f.read(1))[0]
        little_endian = (ei_data == 1)
        endian = '<' if little_endian else '>'
        
        # Read entry point
        f.seek(0x18)  # e_entry for 32-bit
        entry_point = struct.unpack(f'{endian}I', f.read(4))[0]
        
        f.seek(0x1C)  # e_phoff for 32-bit ELF
        e_phoff = struct.unpack(f'{endian}I', f.read(4))[0]
        
        f.seek(0x2A)  # e_phentsize
        e_phentsize = struct.unpack(f'{endian}H', f.read(2))[0]
        
        f.seek(0x2C)  # e_phnum
        e_phnum = struct.unpack(f'{endian}H', f.read(2))[0]
        
        # Read program headers
        for i in range(e_phnum):
            f.seek(e_phoff + i * e_phentsize)
            
            p_type = struct.unpack(f'{endian}I', f.read(4))[0]
            p_offset = struct.unpack(f'{endian}I', f.read(4))[0]
            p_vaddr = struct.unpack(f'{endian}I', f.read(4))[0]
            p_paddr = struct.unpack(f'{endian}I', f.read(4))[0]
            p_filesz = struct.unpack(f'{endian}I', f.read(4))[0]
            p_memsz = struct.unpack(f'{endian}I', f.read(4))[0]
            p_flags = struct.unpack(f'{endian}I', f.read(4))[0]
            
            # PT_LOAD = 1
            if p_type == 1 and p_filesz > 0:
                f.seek(p_offset)
                data = f.read(p_filesz)
                segments.append({
                    'addr': p_paddr,
                    'data': data,
                    'size': p_filesz
                })
    
    return segments, little_endian, entry_point


def create_riscv_jump_bytes(target_addr, current_addr):
    """
    Create a RISC-V JAL instruction as bytes (little-endian).
    Returns 4 bytes for the J instruction.
    """
    offset = target_addr - current_addr
    
    if offset < -1048576 or offset > 1048574:
        raise ValueError(f"Jump offset {offset} out of range for JAL")
    
    imm_21bit = offset & 0x1FFFFF
    
    imm20 = (imm_21bit >> 20) & 0x1
    imm10_1 = (imm_21bit >> 1) & 0x3FF
    imm11 = (imm_21bit >> 11) & 0x1
    imm19_12 = (imm_21bit >> 12) & 0xFF
    
    instr = ((imm20 << 31) | 
             (imm10_1 << 21) | 
             (imm11 << 20) | 
             (imm19_12 << 12) | 
             (0b00000 << 7) | 
             0b1101111)
    
    # Return as little-endian bytes
    return struct.pack('<I', instr)


def write_xheep_hex(segments, output_path, little_endian=True, entry_point=0, 
                    boot_addr=0x180, add_boot_vector=True):
    """Write X-HEEP compatible hex format (byte-oriented, space-separated)."""
    
    # Create byte-level memory image
    memory = {}
    
    for seg in segments:
        addr = seg['addr']
        data = seg['data']
        
        for i, byte in enumerate(data):
            memory[addr + i] = byte
    
    # Add boot vector at 0x180
    if add_boot_vector:
        jump_bytes = create_riscv_jump_bytes(entry_point, boot_addr)
        
        if boot_addr in memory:
            old_bytes = bytes([memory.get(boot_addr + i, 0) for i in range(4)])
            print(f"Overwriting at 0x{boot_addr:08X}: {old_bytes.hex()} -> {jump_bytes.hex()}")
        
        for i, byte in enumerate(jump_bytes):
            memory[boot_addr + i] = byte
        
        print(f"Added boot vector at 0x{boot_addr:08X}: J to 0x{entry_point:08X}")
        print(f"  Bytes: {' '.join(f'{b:02X}' for b in jump_bytes)}")
    
    if not memory:
        print("No data to write!")
        return
    
    # Find contiguous regions
    sorted_addrs = sorted(memory.keys())
    min_addr = sorted_addrs[0]
    max_addr = sorted_addrs[-1]
    
    # Write output in X-HEEP format
    BYTES_PER_LINE = 16
    
    with open(output_path, 'w') as f:
        current_line_start = None
        line_bytes = []
        
        for addr in range(min_addr, max_addr + 1):
            line_start = (addr // BYTES_PER_LINE) * BYTES_PER_LINE
            
            # Check if we need a new line or address marker
            if current_line_start is None or line_start != current_line_start:
                # Write previous line if exists
                if line_bytes:
                    f.write(' '.join(f'{b:02X}' for b in line_bytes) + '\n')
                    line_bytes = []
                
                # Check if there's a gap (need address marker)
                if current_line_start is None or line_start != current_line_start + BYTES_PER_LINE:
                    f.write(f'@{line_start:08X}\n')
                
                current_line_start = line_start
            
            # Add byte (or 00 if not in memory)
            byte = memory.get(addr, 0x00)
            line_bytes.append(byte)
            
            # If we have a full line, write it
            if len(line_bytes) == BYTES_PER_LINE:
                f.write(' '.join(f'{b:02X}' for b in line_bytes) + '\n')
                line_bytes = []
                current_line_start = line_start + BYTES_PER_LINE
        
        # Write any remaining bytes
        if line_bytes:
            f.write(' '.join(f'{b:02X}' for b in line_bytes) + '\n')
    
    print(f"\nWritten to {output_path}")
    print(f"Address range: 0x{min_addr:08X} - 0x{max_addr:08X}")
    print(f"Total bytes: {max_addr - min_addr + 1}")


def main():
    parser = argparse.ArgumentParser(
        description='Convert ELF to X-HEEP hex format'
    )
    parser.add_argument('input', help='Input ELF file')
    parser.add_argument('output', help='Output hex file')
    parser.add_argument('--no-boot-vector', action='store_true',
                        help='Do not add boot vector at 0x180')
    parser.add_argument('--boot-addr', type=lambda x: int(x, 0), default=0x180,
                        help='Boot address (default: 0x180)')
    
    args = parser.parse_args()
    
    try:
        segments, little_endian, entry_point = read_elf_segments(args.input)
        
        if not segments:
            print("No loadable segments found!")
            sys.exit(1)
        
        print(f"ELF entry point: 0x{entry_point:08X}")
        print(f"Found {len(segments)} segment(s):")
        for i, seg in enumerate(segments):
            print(f"  [{i}] addr=0x{seg['addr']:08X} size={seg['size']} bytes")
        print()
        
        write_xheep_hex(
            segments, 
            args.output, 
            little_endian,
            entry_point,
            args.boot_addr,
            not args.no_boot_vector
        )
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
