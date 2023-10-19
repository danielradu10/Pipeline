`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/29/2023 05:03:18 PM
// Design Name: 
// Module Name: top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top(input clk, input res);

    wire [31:0] cm_addr;
    wire[31:0] instr;
    wire RegWrite;
    
    wire PC_Src;
    wire [31:0] Suma_Out;
    
    wire [4:0] out_instr11_7;
    wire [31:0] ReadData1;
    wire  [31:0] ReadData2;
    wire  [31:0] dc_out; 
    
    
    //lucruri pe care le plimb
    wire [31:0] iesire_imm;
    wire [31:0] pc_out_from_ID;
    wire [2:0] alu_sig; 
    wire [2:0] mem_sig;
    wire [1:0] reg_sig;
    wire [3:0] instruction_30_14_12;
    wire [4:0] instructio11_7;
    wire [6:0] funct7;
    wire [2:0] funct3;
    
    
    wire [2:0] out_mem_signals;
    wire [1:0] out_write_signals;
    wire [31:0] Suma;
    wire [31:0] ALUOut;
    wire [31:0] out_readData2;
    wire [4:0] out_in_11_7;
    wire Zero;
    
    wire [2:0] out_out_write_signals;
    wire [31:0] out_data_memory;
    wire [31:0] out_ALUOut;
    wire [31:0] out_out_instr_11_7;
    
    
   
    
    //code_memory CM(.addr(cm_addr[7:0]), .dout(instr));
    
//    assign PC_Src = 0;
//    assign Suma_Out = 0;
    
    instruction_fetch IF(
        .clk(clk),
        .res(res),
        .pc(cm_addr),
        
        .instructiune(instr),
        .PC_Src(PC_Src),
        .PC_suma(Suma_Out)
       
    );
    
    instruction_decode ID(
        .clk(clk),
        .reset(res),
        .pc(cm_addr),
        .instruction(instr),
        .RegWrite(RegWrite),
        .instruction11_7(out_instr11_7),
        .dc(dc_out),
        .ReadData1(ReadData1),
        .ReadData2(ReadData2),
        .pc_out(pc_out_from_ID),
        .iesire_imm(iesire_imm),
        .instr30_14_12(instruction_30_14_12),
        .instr11_7(instructio11_7),
        .alu_signals(alu_sig),
        .mem_signals(mem_sig),
        .write_signals(reg_sig),
        .funct7(funct7),
        .funct3(funct3)
    );
    
    execute EX(
    .clk(clk),
    .alu_signals(alu_sig),
    .mem_signals(mem_sig),
    .write_signals(reg_sig),
    .pc(pc_out_from_ID),
    .imm_gen(iesire_imm),
    .ReadData1(ReadData1),
    .ReadData2(ReadData2),
    .instruction_30_14_12(instruction_30_14_12),
    .instruction_11_7(instructio11_7),
    .funct7(funct7),
    .funct3(funct3),
    .out_mem_signals(out_mem_signals),
    .out_write_signals(out_write_signals),
    .Suma(Suma),
    .AluOut(ALUOut),
    .out_ReadData2(out_readData2),
    .out_instr_11_7(out_in_11_7),
    .Zero(Zero)
   
    );
    
    memory_access M(
    .clk(clk),
    .reset(res),
    .mem_signals(out_mem_signals),
    .write_signals(out_write_signals),
    .Suma(Suma),
    .Zero(Zero),
    .ALUOut(ALUOut),
    .ReadData2(out_readData2),
    .instr_11_7(out_in_11_7),
    
    .out_wrt_signals(out_out_write_signals),
    .ReadDataMemory(out_data_memory),
    .out_ALUOut(out_ALUOut),
    .out_instr_11_7(out_out_instr_11_7),
    .jump_condition(PC_Src),
    
    .out_Suma(Suma_Out)
   
    );
    
    
    write_back WB(
        .clk(clk),
        .write_signals(out_out_write_signals),
        .ALUResult(out_ALUOut),
        .instr_11_7(out_out_instr_11_7),
        .RegWrite(RegWrite),
        .WriteRegister(out_instr11_7),
        .WriteData(dc_out)
    
    );  
    
endmodule






module instruction_fetch(input clk, input res, output reg[31:0] pc, output [31:0] instructiune, input PC_Src, input [31:0] PC_suma);
    
    
    parameter 	DATA_LENGTH  = 32;
    parameter 	MEM_SIZE     = 256; 
    
    reg [DATA_LENGTH / 4 -1:0] mem [0:MEM_SIZE * 4 -1];

  
   
    
    initial begin
        $readmemh( "memory_code.mem", mem);
    end
    
   
    
    always @(posedge clk)
        if(res)
            pc<=32'h0000_0000;
            
        else
            if(PC_Src == 0)
                 pc<=pc+4;
            else
                pc<=PC_suma;
            
            
    assign instructiune = { mem[pc+3], mem[pc+2], mem[pc+1], mem[pc] };
endmodule


module instruction_decode(
        
        //generale
        input clk, 
        input reset,
        input [31:0] pc,
        
        //pentru imm si control
        input [31:0] instruction,
        
        // pentru register file
        input RegWrite,
        input instruction11_7,
        input [31:0] dc,
        
        //iesiri
        
        //din register file
        output  reg [31:0] ReadData1,
        output  reg [31:0] ReadData2,
        
        output [31:0] pc_out, 
        output reg [31:0] iesire_imm,
        
        //din instructiune
        output reg [3:0] instr30_14_12,
        output reg [4:0] instr11_7,
        
        //semnalele lui control
        output reg [2:0] alu_signals,
        output reg [2:0] mem_signals,
        output reg [1:0] write_signals,
         //funct for alu
        output reg [6:0] funct7,
        output reg [2:0] funct3);
        
       
        
        
       
        
        
        reg [31:0] mem [0:31];

      assign pc_out = pc;
       always @(posedge clk) begin
         ReadData1 <= (instruction[19:15]!= 5'b00000) ? mem[instruction[19:15]] : 32'b0; 
         ReadData2 <= (instruction[24:20]!= 5'b00000) ? mem[instruction[24:20]] : 32'b0;
         instr30_14_12 <= {instruction[30], instruction[14:12]};
         instr11_7 <= {instruction[11:7]};
         funct7<=instruction[31:25];
         funct3<=instruction[14:12];
         
         //pc_out    <=  pc;            
        if (reset == 1'b1) begin
            mem[0] <= 0; mem[1] <= 0; mem[2] <= 0; mem[3] <= 0; mem[4] <= 0; mem[5] <= 0; mem[6] <= 0; mem[7] <= 0; 
            mem[8] <= 0; mem[9] <= 0; mem[10] <= 0; mem[11] <= 0; mem[12] <= 0; mem[13] <= 0; mem[14] <= 0; mem[15] <= 0;
            mem[16] <= 0; mem[17] <= 0; mem[18] <= 0; mem[19] <= 0; mem[20] <= 0; mem[21] <= 0; mem[22] <= 0; mem[23] <= 0;
            mem[24] <= 0; mem[25] <= 0; mem[26] <= 0; mem[27] <= 0; mem[28] <= 0; mem[29] <= 0; mem[30] <= 0; mem[31] <= 0;
        end else begin 
            if (RegWrite == 1'b1) begin
                mem[instruction11_7] <= dc;
            end
            end
            
            case(instruction[6:0])
            7'b0000011,
            7'b0001111,
            7'b0011011,
            7'b1100111,
            7'b1110011,
            7'b0010011: iesire_imm = { {20{instruction[31]}}, instruction[31:20]};
            7'b0100011: iesire_imm = { {20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            7'b1100011: iesire_imm = { {20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};            
            7'b1101111: iesire_imm = { {12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:25], instruction[11:8], 1'b0};            
            7'b0010111,
            7'b0110111: iesire_imm = { instruction[31:12], {12{1'b0}}}; 
            default:
                iesire_imm = 32'h0000_0000;  
            endcase
            
            casex(instruction[6:0])
            7'b0110011: {alu_signals, mem_signals, write_signals} = 8'b1_0_0_0_0_0_1_0;  // R
            7'b0000011: {alu_signals, mem_signals, write_signals} = 8'b0_0_1_0_1_0_11;  // lw
            7'b0100011: {alu_signals, mem_signals, write_signals} = 8'b0_0_1_0_0_1_0x;  // sw
            7'b1100011: {alu_signals, mem_signals, write_signals} = 8'b0_1_0_1_0_0_0x;  // beq
            
            7'b0010011: {alu_signals, mem_signals, write_signals} = 8'b1_1_1_0_0_0_1_0;  // I
            
            default:
                {alu_signals, mem_signals, write_signals} = 8'b0_0_0_0_0_0_00;
        endcase
   
            
        end
       
endmodule



module execute(
        input clk,
         input [2:0] alu_signals, 
         input [2:0] mem_signals, 
         input [1:0] write_signals, 
         input [31:0] pc, 
         input [31:0] imm_gen,
         input [31:0] ReadData1,
         input [31:0] ReadData2,
         input [3:0] instruction_30_14_12,
         input [4:0] instruction_11_7,
         input [6:0] funct7,
         input [2:0] funct3,
         
         
         //iesirile
         output reg [2:0] out_mem_signals,
         output reg [1:0]  out_write_signals,
         output reg [31:0] Suma,
         output reg [31:0] AluOut,
         output [31:0] out_ReadData2,
         output [4:0] out_instr_11_7,
         output Zero
         );
     reg [3:0] alu_ctrl;   
     always @(posedge clk) begin
        out_mem_signals <= mem_signals;
        out_write_signals <= write_signals;
        Suma <= pc + imm_gen;
        
        casex({alu_signals[2:1], funct7, funct3})
             12'b00_xxxxxxx_xxx: alu_ctrl = 4'b0010; // lw sw
             12'b01_xxxxxxx_xxx: alu_ctrl = 4'b0110; // beq
                                
             12'b10_0000000_000: alu_ctrl = 4'b0010; // add
             12'b10_0100000_000: alu_ctrl = 4'b0110; // sub            
             12'b10_0000000_111: alu_ctrl = 4'b0000; // and
             12'b10_0000000_110: alu_ctrl = 4'b0001; // or
           
                          
             12'b11_xxxxxxx_000: alu_ctrl = 4'b0010; // addi
             12'b11_xxxxxxx_111: alu_ctrl = 4'b0110; // andi
             12'b11_xxxxxxx_110: alu_ctrl = 4'b0000; // ori
             12'b11_xxxxxxx_100: alu_ctrl = 4'b0001; // xori   
            
            
                
   
            default:
                alu_ctrl = 4'b0000;
        endcase
        
        if(alu_signals[0] == 0) begin
        case(alu_ctrl)
            4'b0010: AluOut = ReadData1 + ReadData2;
            4'b0110: AluOut = ReadData1 - ReadData2;
            4'b0000: AluOut = ReadData1 & ReadData2;
            4'b0001: AluOut = ReadData1 | ReadData2;
            4'b0011: AluOut = ReadData1 ^ ReadData2;
            default:
                AluOut = 32'hFFFF_FFFF;
        endcase
        end else begin
         case(alu_ctrl)
            4'b0010: AluOut = ReadData1 + imm_gen;
            4'b0110: AluOut = ReadData1 & imm_gen;
            4'b0000: AluOut = ReadData1 | imm_gen;
            4'b0001: AluOut = ReadData1 ^ imm_gen;
           
            default:
                AluOut = 32'hFFFF_FFFF;
        endcase
        
        end
        
        
      
     end
     
     
//     always @(alu_signals[2:1] or funct7 or funct3) begin
//      casex({alu_signals[2:1], funct7, funct3})
//             12'b00_xxxxxxx_xxx: alu_ctrl = 4'b0010; // lw sw
//             12'b01_xxxxxxx_xxx: alu_ctrl = 4'b0110; // beq
                                
//             12'b10_0000000_000: alu_ctrl = 4'b0010; // add
//             12'b10_0100000_000: alu_ctrl = 4'b0110; // sub            
//             12'b10_0000000_111: alu_ctrl = 4'b0000; // and
//             12'b10_0000000_110: alu_ctrl = 4'b0001; // or
           
                          
//             12'b11_xxxxxxx_000: alu_ctrl = 4'b0010; // addi
//             12'b11_xxxxxxx_111: alu_ctrl = 4'b0110; // andi
//             12'b11_xxxxxxx_110: alu_ctrl = 4'b0000; // ori
//             12'b11_xxxxxxx_100: alu_ctrl = 4'b0001; // xori   
            
            
                
   
//            default:
//                alu_ctrl = 4'b0000;
//        endcase
//        end
     
//     always @(ReadData1 or ReadData2 or alu_ctrl) begin
//        if(alu_signals[0] == 0) begin
//        case(alu_ctrl)
//            4'b0010: AluOut = ReadData1 + ReadData2;
//            4'b0110: AluOut = ReadData1 - ReadData2;
//            4'b0000: AluOut = ReadData1 & ReadData2;
//            4'b0001: AluOut = ReadData1 | ReadData2;
//            4'b0011: AluOut = ReadData1 ^ ReadData2;
//            default:
//                AluOut = 32'hFFFF_FFFF;
//        endcase
//        end else begin
//         case(alu_ctrl)
//            4'b0010: AluOut = ReadData1 + imm_gen;
//            4'b0110: AluOut = ReadData1 & imm_gen;
//            4'b0000: AluOut = ReadData1 | imm_gen;
//            4'b0001: AluOut = ReadData1 ^ imm_gen;
           
//            default:
//                AluOut = 32'hFFFF_FFFF;
//        endcase
        
//        end
//    end
    
    assign out_ReadData2 = ReadData2;
    assign out_instr_11_7 = instruction_11_7;
    assign Zero = ~(|AluOut);
     //always @(ReadData1 or ReadData2 or 
    
endmodule


module memory_access(
    input clk, 
    input reset,
    input [2:0] mem_signals, 
    input [1:0] write_signals,
    input [31:0] Suma,
    input Zero,
    input [31:0] ALUOut,
    input [31:0] ReadData2,
    input [4:0] instr_11_7,
    
    //ce scoate
    
    output reg [1:0] out_wrt_signals,
    output reg [31:0] ReadDataMemory,
    output reg [31:0] out_ALUOut,
    output reg [4:0] out_instr_11_7,
    output jump_condition,
    output reg [31:0] out_Suma
    
   
    );
    
    parameter 	DATA_LENGTH  = 32;
    parameter 	MEM_SIZE     = 256; 
    
    reg [DATA_LENGTH / 4 - 1:0] mem [0:MEM_SIZE * 4 - 1];
    
    initial begin
        $readmemh( "memory_data.mem", mem);
    end
    
   
    always @(posedge clk) begin
        out_wrt_signals <= write_signals;
        out_ALUOut <= ALUOut;
        out_instr_11_7 <= instr_11_7;
        out_Suma <= Suma;
         if(reset == 1) begin
                out_Suma = 0;
                //jump_condition = 0;
         end
    
        if(mem_signals[0] == 1) begin
            { mem[ALUOut+3], mem[ALUOut+2], mem[ALUOut+1], mem[ALUOut] } <= ReadData2;
        end
        if(mem_signals[1] == 1) begin
            ReadDataMemory <=  { mem[ALUOut+3], mem[ALUOut+2], mem[ALUOut+1], mem[ALUOut] };
        
        end
       
       
    
    end
    
     assign jump_condition = Zero & mem_signals[2];
    
endmodule


module write_back(
    input clk,
    input [1:0] write_signals,
    input [31:0] ReadData,
    input [31:0] ALUResult,
    input [4:0] instr_11_7,
    
    output reg RegWrite,
    output reg[4:0] WriteRegister,
    output reg[31:0] WriteData
 
);

    always @(posedge clk) begin
        
        casex(write_signals)
            2'b10: begin
                   WriteRegister <= instr_11_7;
                   WriteData <= ALUResult; 
                   end    
            2'b11: begin 
                   WriteRegister <= instr_11_7;
                   WriteData <= ReadData;  
                   end
        endcase
        RegWrite <= write_signals[1];
    
    end
    
    
        



endmodule
    