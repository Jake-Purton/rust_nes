use std::collections::HashMap;
use crate::{color, opcodes};

const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xfd;

#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect_X,
    Indirect_Y,
    NoneAddressing,
    Accumulator,
}

const CARRY: u8 = 0b00000001;
const ZERO: u8 = 0b00000010;
const INTERRUPT_DISABLE: u8 = 0b00000100;
const DECIMAL_MODE: u8 = 0b00001000;
const BREAK: u8 = 0b00010000;
const BREAK2: u8 = 0b00100000;
const OVERFLOW: u8 = 0b01000000;
const NEGATIVE: u8 = 0b10000000;

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub stack_pointer: u8,

    /// # Status Register (P) http://wiki.nesdev.com/w/index.php/Status_flags
    ///
    ///  7 6 5 4 3 2 1 0
    ///  N V _ B D I Z C
    ///  | |   | | | | +--- Carry Flag
    ///  | |   | | | +----- Zero Flag
    ///  | |   | | +------- Interrupt Disable
    ///  | |   | +--------- Decimal Mode (not used on NES)
    ///  | |   +----------- Break Command
    ///  | +--------------- Overflow Flag
    ///  +----------------- Negative Flag
    ///

    pub status: u8,
    pub program_counter: u16,
    memory: [u8; 0xFFFF]
}

trait Mem {
    fn mem_read(&self, addr: u16) -> u8; 

    fn mem_write(&mut self, addr: u16, data: u8);
    
    fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos) as u16;
        let hi = self.mem_read(pos + 1) as u16;
        (hi << 8) | (lo as u16)
    }

    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.mem_write(pos, lo);
        self.mem_write(pos + 1, hi);
    }
}

impl Mem for CPU {
    
    fn mem_read(&self, addr: u16) -> u8 { 
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) { 
        self.memory[addr as usize] = data;
    }
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            status: 0,
            program_counter: 0,
            memory: [0; 0xFFFF],
            stack_pointer: STACK_RESET,
        }
    }

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {

        match mode {
            AddressingMode::Immediate => self.program_counter,

            AddressingMode::ZeroPage  => self.mem_read(self.program_counter) as u16,
            
            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),

            AddressingMode::Accumulator => self.register_a as u16,
          
            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_x) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read(self.program_counter);
                let addr = pos.wrapping_add(self.register_y) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_x as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                let addr = base.wrapping_add(self.register_y as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);

                let ptr: u8 = (base as u8).wrapping_add(self.register_x);
                let lo = self.mem_read(ptr as u16);
                let hi = self.mem_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);

                let lo = self.mem_read(base as u16);
                let hi = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                deref
            }
           
            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported", mode);
            }
        }

    }

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read(addr);

        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read(addr);

        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    fn stack_pop(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        self.mem_read((STACK as u16) + self.stack_pointer as u16)
    }

    fn stack_pop_u16(&mut self) -> u16 {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;

        hi << 8 | lo
    }


    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn txa(&mut self) {
        self.register_a = self.register_x;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn lsra (&mut self) {
        if self.register_a & CARRY > 0 {
            self.sec()
        } else {
            self.clc()
        }

        self.register_a = self.register_a >> 1;

        self.update_zero_and_negative_flags(self.register_a)
    }

    fn asl(&mut self, mode: &AddressingMode) {

        if matches!(mode, AddressingMode::Accumulator) {

            if self.register_a & 0b1000_0000 > 0 {
                self.sec()
            } else {
                self.clc()
            }

            self.register_a = self.register_a << 1;

            self.update_zero_and_negative_flags(self.register_a)

        } else {

            let addr = self.get_operand_address(mode);
            let value = self.mem_read(addr);
    
    
            if value & 0b1000_0000 > 0 {
                self.sec()
            } else {
                self.clc()
            }

            let result = value << 1;
            self.update_zero_and_negative_flags(result);
            self.mem_write(addr, result);

        }
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        if result == 0 {
            self.status = self.status | 0b0000_0010;
        } else {
            self.status = self.status & 0b1111_1101;
        }

        if result & 0b1000_0000 != 0 {
            self.status = self.status | 0b1000_0000;
        } else {
            self.status = self.status & 0b0111_1111;
        }
    }

    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn dex(&mut self) {
        self.register_x = self.register_x.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read(addr);

        let c = if self.status & 1 == 1 {
            1
        } else {
            0
        };

        let a = self.register_a as u16  + value as u16 + c;

        let carry = ((a & 0b0000_0001_0000_0000) >> 8) as u8;

        let sum = (a & 0b1111_1111) as u8;

        self.status = self.status & 0b1111_1110;
        self.status = self.status | carry;

        self.register_a = sum;

        self.update_zero_and_negative_flags(self.register_a);
        

    }

    fn sec (&mut self) {
        self.status = self.status | CARRY;
    }

    fn clc (&mut self) {
        self.status = self.status & 0b1111_1110;
    }

    fn ora (&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read(addr);

        self.register_a = value | self.register_a;

        self.update_zero_and_negative_flags(self.register_a)
    }

    fn xor (&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read(addr);

        self.register_a = value ^ self.register_a;

        self.update_zero_and_negative_flags(self.register_a)
    }

    fn and (&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read(addr);

        self.register_a = value & self.register_a;

        self.update_zero_and_negative_flags(self.register_a)
    }

    fn cmp (&mut self, mode: &AddressingMode) {

        let addr = self.get_operand_address(&mode);
        
        let a = self.register_a;
        let m = self.mem_read(addr);

        if a >= m {
            self.sec();
        } else {
            self.clc();
        }
        if a == m {
            self.clear_zero();
        } else {
            self.set_zero();
        }
    }

    fn cpx (&mut self, mode: &AddressingMode) {

        let addr = self.get_operand_address(&mode);
        
        let a = self.register_x;
        let m = self.mem_read(addr);

        if a >= m {
            self.sec();
        } else {
            self.clc();
        }
        if a == m {
            self.clear_zero();
        } else {
            self.set_zero();
        }
    }

    fn rola (&mut self) {
        let carry = if self.status & 1 == 1 {
            1
        } else {
            0
        };

        if self.register_a & 0b1000_0000 > 0 {
            self.sec()
        } else {
            self.clc()
        }

        self.register_a = (self.register_a << 1) + carry;

        self.update_zero_and_negative_flags(self.register_a);
        
    }

    fn bit (&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read(addr);

        if value & self.register_a == 0 {
            self.set_zero()
        } else {
            self.clear_zero()
        }

        if value & 0b1000_0000 > 0 {
            self.set_negative()
        } else {
            self.clear_negative()
        }

        if value & 0b0100_0000 > 0 {
            self.set_overflow()
        } else {
            self.clear_overflow()
        }

    }

    fn set_overflow (&mut self) {
        self.status = self.status | 0b0100_0000;
    }

    fn clear_overflow (&mut self) {
        self.status = self.status & 0b1011_1111;
    }

    fn set_zero (&mut self) {
        self.status = self.status | 0b0000_0010;
    }

    fn clear_zero (&mut self) {
        self.status = self.status & 0b1111_1101;
    }

    fn set_negative (&mut self) {
        self.status = self.status | 0b1000_0000;
    }

    fn clear_negative (&mut self) {
        self.status = self.status & 0b0111_1111;
    }

    fn rol (&mut self, mode: &AddressingMode) {

        let addr = self.get_operand_address(&mode);
        let value = self.mem_read(addr);

        let carry = if self.status & 1 == 1 {
            1
        } else {
            0
        };

        if value & 0b1000_0000 > 0 {
            self.sec()
        } else {
            self.clc()
        }

        let new_val = (value << 1) + carry;

        self.mem_write(addr, new_val)

    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run()
    }

    pub fn load(&mut self, program: Vec<u8>) {
        // self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        // self.mem_write_u16(0xFFFC, 0x8000);

        self.memory[0x0600..(0x0600 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(0xFFFC, 0x0600);
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.status = 0;

        self.program_counter = self.mem_read_u16(0xFFFC);
    }

    fn jump (&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);

        self.program_counter = addr;
    }

    fn branch(&mut self, condition: bool) {
        if condition {
            let jump: i8 = self.mem_read(self.program_counter) as i8;
            let jump_addr = self
                .program_counter
                .wrapping_add(1)
                .wrapping_add(jump as u16);

            self.program_counter = jump_addr;
        }
    }

    fn add_to_register_a(&mut self, data: u8) {
        let sum = self.register_a as u16
            + data as u16
            + (if self.status & CARRY > 0  {
                1
            } else {
                0
            }) as u16;

        let carry = sum > 0xff;

        if carry {
            self.sec();
        } else {
            self.clc();
        }

        let result = sum as u8;

        if (data ^ result) & (result ^ self.register_a) & 0x80 != 0 {
            self.set_overflow();
        } else {
            self.clear_overflow();
        }

        self.register_a = result;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn stack_push(&mut self, data: u8) {
        self.mem_write((STACK as u16) + self.stack_pointer as u16, data);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1)
    }

    fn sbc (&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let data = self.mem_read(addr);
        self.add_to_register_a(((data as i8).wrapping_neg().wrapping_sub(1)) as u8);
    }

    fn stack_push_u16(&mut self, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;
        self.stack_push(hi);
        self.stack_push(lo);
    }

    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        data = data.wrapping_add(1);
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
    }

    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read(addr);
        data = data.wrapping_sub(1);
        self.mem_write(addr, data);
        self.update_zero_and_negative_flags(data);
    }
    
    fn read_screen_state(&self, frame: &mut [u8; 32 * 3 * 32]) -> bool {
        let mut frame_idx = 0;
        let mut update = false;
        for i in 0x0200..0x600 {
            let color_idx = self.mem_read(i as u16);

            if color_idx == 1 {
                println!("happy")
            }

            let (b1, b2, b3) = color(color_idx).rgb();
            if frame[frame_idx] != b1 || frame[frame_idx + 1] != b2 || frame[frame_idx + 2] != b3 {
                frame[frame_idx] = b1;
                frame[frame_idx + 1] = b2;
                frame[frame_idx + 2] = b3;
                update = true;
            }
            frame_idx += 3;
        }
        update
    }

    pub fn run(&mut self) {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP;
        let mut frame: [u8; 32 * 3 * 32] = [0; 32 * 3 * 32];

        loop {
            let code = self.mem_read(self.program_counter);
            self.program_counter += 1;
            let program_counter_state = self.program_counter;

            let opcode = opcodes.get(&code).expect(&format!("OpCode {:x} is not recognized", code));

            println!("pc: {}, acc: {:b}, x: {}, y: {}, status: {:b}, opcode: {}, {:x}", program_counter_state - 0x600, self.register_a, self.register_x, self.register_y, self.status, opcode.mnemonic, opcode.code);
            
            // println!("code: {}", opcode.code);

            match code {
                // LDA
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                }

                /* BNE */
                0xd0 => {
                    self.branch(!(self.status & ZERO > 0));
                }

                /* BVS */
                0x70 => {
                    self.branch((self.status & OVERFLOW) > 0);
                }

                /* BVC */
                0x50 => {
                    self.branch(!(self.status & OVERFLOW > 0));
                }

                /* BPL */
                0x10 => {
                    self.branch(!(self.status & NEGATIVE > 0));
                }

                /* BMI */
                0x30 => {
                    self.branch(self.status & NEGATIVE > 0);
                }

                /* BEQ */
                0xf0 => {
                    self.branch(self.status & ZERO > 0);
                }

                /* BCS */
                0xb0 => {
                    self.branch(self.status & CARRY > 0);
                }

                /* BCC */
                0x90 => {
                    self.branch(!(self.status & CARRY > 0));
                }

                /* SBC */
                0xe9 | 0xe5 | 0xf5 | 0xed | 0xfd | 0xf9 | 0xe1 | 0xf1 => {
                    self.sbc(&opcode.mode);
                }

                // CPX
                0xe0 | 0xe4 | 0xec => self.cpx(&opcode.mode),

                // LDX
                0xa2 | 0xae | 0xbe | 0xa6 | 0xb6 => self.ldx(&opcode.mode),

                /* STA */
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }

                // ADC
                0x69 | 0x65 | 0x75 | 0x6d | 0x7d | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                }

                // INC 
                0xe6 | 0xf6 | 0xee | 0xfe => self.inc(&opcode.mode),
                // DEC 
                0xc6 | 0xd6 | 0xce | 0xde => self.dec(&opcode.mode),

                // jump
                0x4c => self.jump(&opcode.mode),

                /* JSR */
                0x20 => {
                    self.stack_push_u16(self.program_counter + 2 - 1);
                    let target_address = self.mem_read_u16(self.program_counter);
                    self.program_counter = target_address;
                    println!("jump to {}", target_address - 0x600);

                }

                // compare
                0xc9 | 0xc5 => self.cmp(&opcode.mode),

                // return from subroutine (rst)
                0x60 => {
                    let address = self.stack_pop_u16();

                    // println!("return to {address}");
                    self.program_counter = address + 1;
                }



                0x24 => {
                    self.bit(&opcode.mode)
                }

                // logical and (AND)
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode)
                }
                
                // Transfer accumulator to x
                0xAA => self.tax(),

                0x8A => self.txa(),

                // Increment x
                0xe8 => self.inx(),

                // DEX
                0xca => self.dex(),

                // Arithmetic shift left
                0x06 => self.asl(&opcode.mode),

                // LSR a
                0x4a => self.lsra(),

                // Set carry flag to 1
                0x38 => self.sec(),

                // rol with opcode mode
                0x26 => {
                    self.rol(&opcode.mode)
                },
                // rol accumulator
                0x2a => {
                    self.rola()
                },

                /* STX */
                0x86 | 0x96 | 0x8e => {
                    let addr = self.get_operand_address(&opcode.mode);
                    self.mem_write(addr, self.register_x);
                }

                // accumulator or
                0x09 | 0x05 | 0x15 | 0x0d | 0x1d | 0x19 | 0x01 | 0x11 => self.ora(&opcode.mode),

                // accumulator xor
                0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 => self.xor(&opcode.mode),


                // clear carry
                0x18 => self.clc(),

                // BRK
                0x00 => return,
                _ => todo!(),
            }        

            if program_counter_state == self.program_counter {
                self.program_counter += (opcode.len - 1) as u16;
            }

            self.read_screen_state(&mut frame);
            // println!("{:?}", frame);
            
        }
    }
}


#[cfg(test)]
mod test {
    use super::*;

    

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.register_a, 5);
        assert!(cpu.status & 0b0000_0010 == 0);
        assert!(cpu.status & 0b1000_0000 == 0);
    }

    #[test]
    fn test_0x06_left_shift() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0b1000_1111, 0x85, 0xff, 0x06, 0xff, 0xa5, 0xff]);
        assert_eq!(cpu.register_a, 0b0001_1110);
        assert!(cpu.status & CARRY > 0);
    }

    #[test]
    fn test_0x24_bit() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0b1000_1111, 0x85, 0x00, 0xA9, 0b0001_0100, 0x24, 0x00]);
        assert!(cpu.status & 0b0100_0000 == 0);
        assert!(cpu.status & 0b1000_0000 > 0);
        assert!(cpu.status & 0b0000_0010 > 0);
    }


    #[test]
    fn test_0x2a_rotate_left() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0x38, 0x2a, 0x2a, 0x2a]);
        assert_eq!(cpu.register_a, 0b0000_0100);
        assert!(cpu.status & 0b0000_0001 == 0);

        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0b0010_0000,0x38, 0x2a, 0x2a, 0x2a]);
        assert_eq!(cpu.register_a, 0b0000_0100);
        assert!(cpu.status & 0b0000_0001 == 1);
    }

    #[test]
    fn test_0x38_set_carry() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0x38, 0x00]);
        assert!(cpu.status & 0b0000_0001 == 0b0000_0001);
    }


    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x0A,0xaa, 0x00]);

        assert_eq!(cpu.register_x, 10)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.register_x = 0xff;
        cpu.load_and_run(vec![0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.register_x, 1)
    }

    #[test]
    fn test_adc_overflow() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x01, 0x69, 0xff,]);

        assert_eq!(cpu.register_a, 0);
        assert_eq!(cpu.status, 0b00000011);
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.register_a, 0x55);
    }
}