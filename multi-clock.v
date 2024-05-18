// Dunia Jaser 1201345
// Alaa Shaheen 1200049
//Shatha Khdair 1200525
module InstructionMemory(data_out, address);		
	input[31:0] address;
	output reg [31:0] data_out;		 
	
	parameter MEM_DEPTH = 256;
	reg [31:0] instructions[0:MEM_DEPTH-1]; 

	
	parameter AND=6'b000000,ADD=6'b000001, SUB=6'b000010;//R-Type Instructions
	parameter ANDI=6'b000011, ADDI=6'b000100, LW=6'b000101, LWPOI=6'b000110, SW=6'b000111, BGT=6'b001000, BLT=6'b001001 ,BEQ=6'b001010, BNE=6'b001011;//I-Type Instructions
	
	parameter JMP=6'b001100, CALL=6'b001101 , RET=6'b001110;//J-Type Instructions
	parameter PUSH=6'b001111, POP=6'b010000;//S-Type Instructions
	
	parameter R0=4'b0, R1=4'b0001, R2=4'b0010, R3=4'b0011, R4=4'b0100, R5=4'b0101, R6=4'b0110, R7=4'b0111, R8=4'b1000, R9=4'b1001;		
	
	parameter R10=4'b1010, R11=4'b1011, R12=4'b1100, R13=4'b1101, R14=4'b1110, R15=4'b1111;	 
	
	parameter mode0= 2'b00, mode1= 2'b01;
	initial 
		begin											   
		instructions[0] = {AND,R1, R2, R3, 14'b00001010110000};	// AND R1, R2, R3
		instructions[1] = {ADD,R4, R9, R10, 14'b00001010110000};// ADD,R4, R9, R10	   
		instructions[2] = {ADDI, R6, R8,16'b0000000000000011, mode0};  // ADDI, R6, R8, 3 , no increment/decrement of the base register	
		instructions[3] = {ANDI, R1, R2, 16'b0101010101010101,mode1}; // ANDI R1, R2, Immediate
		instructions[4] = {ADDI, R3, R4, 16'b0011001100110011,mode0};		// ADDI R3, R4, Immediate
		instructions[5] = {LW, R5, 26'b00000000000000000000000001}; // LW R5, Address
		instructions[6] = {JMP, 26'b00000000000000000000000010}; //  JMP Address  
		instructions[7] = {CALL, 26'b00000000000000000000000011}; //  CALL Address		 
		instructions[8] = {PUSH, R10, 22'b0}; // PUSH R10
		instructions[9] = {POP, R11, 22'b0}; // POP R11	   
		// R-Type Instructions
		instructions[10] = {AND, R2, R3, R4, 14'b0}; // AND R2, R3, R4
		instructions[11] = {ADD, R7, R8, R9, 14'b0}; // ADD R7, R8, R9
		instructions[12] = {SUB, R1, R5, R6, 14'b0}; // SUB R1, R5, R6
		// I-Type Instructions
		instructions[13] = {ANDI, R4, R5, 16'b0000111100001111,mode0}; // ANDI R4, R5, Immediate
		instructions[14] = {ADDI, R6, R7, 16'b0001000100010001, mode0}; // ADDI R6, R7, Immediate
		instructions[15] = {LW, R8, 26'b00000000000000000000000100}; // LW R8, Address	
		// J-Type Instructions
		instructions[16] = {JMP, 26'b00000000000000000000000101}; // JMP Address
		instructions[17] = {CALL, 26'b00000000000000000000000110}; // CALL Address
		// S-Type Instructions
		instructions[18] = {PUSH, R12, 22'b0}; // PUSH R12
		instructions[19] = {POP, R13, 22'b0}; // POP R13
		end		
		assign data_out = instructions[address];	
		
endmodule 	  

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module reg_ir(
		input clk,
		input rst,
		input IRWrite,
		input [31:0] InstrIn,
		output reg [5:0] Opcode,
		output reg [3:0] RS1,
		output reg [3:0] RS2,
		output reg [3:0] RW,
		output reg [15:0] Imm,	
		output reg [1:0] Mode,
		output reg [25:0] Jump_Imm
    );
	 
	always @(posedge clk)
		begin
		if (rst)
			begin
			Opcode      <= 6'd0;
			RS1 		<= 4'd0;
			RS2			<= 4'd0;
			RW 			<= 4'd0;
			Imm         <= 16'd0;
			Jump_Imm    <= 26'd0;	
			Mode        <= 2'd0;
			end
		else if (IRWrite)
			begin
			Opcode      <= InstrIn[31:26];
			RS1 		<= InstrIn[21:18];
			RS2 		<= InstrIn[17:14];
			RW			<= InstrIn[25:22];
			Imm         <= InstrIn[17:2];
			Jump_Imm    <= InstrIn[25:0];	
			Mode    	<= InstrIn[1:0];
			end
		end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	

module reg_pc(
		input clk,
		input rst,
		input [31:0] newPC,
		input PCWrite,
		output reg [31:0] PC
    );
	 
	always @(posedge clk)
		begin
		if (rst)
			begin
			PC <= 32'h00000000;
			end
		else if (PCWrite)
			begin
			PC <= newPC;
			end
		end

endmodule	 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
module RegisterFile(
	input [3:0]rs1, rs2, rd,
	input clk, writeSignal,
	input [31:0]data,
	
	output reg [31:0]dataOut1, dataOut2
    );
	 
	 reg [31:0]registers[0:15];
	 
	 integer i;
	 initial begin
		for(i=0;i<31;i=i+1) begin
			registers[i] = 0;
		end
	 end
	 
	 always@(posedge clk)
	 begin
	 
		 if(writeSignal == 1 & rd != 0)
			 begin
				registers[rd] = data;
			 end
			 
		 dataOut1 = registers[rs1];
		 dataOut2 = registers[rs2];
	 
	 end


endmodule	 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
module DataMemory( data_out,     // Output data from the memory
	  		data_in,   // Input data into the memory
            Address,     // Address of data to be read/written
            MemW,    // When high, causes write to take place on posedge	   
			MemRd,
            Clk);        // Clock
                          
    parameter DATA_WIDTH = 32;
    parameter ADDRESS_WIDTH = 32; 
                          
    input  Clk, MemW, MemRd;
    input  [DATA_WIDTH-1:0] data_in;
    input  [ADDRESS_WIDTH-1:0] Address;
    output [DATA_WIDTH-1:0] data_out;
    
    reg [DATA_WIDTH-1:0] mem_data [0:DATA_WIDTH-1];
    
    // for loop initializes all registers to 0, no need to rst
    integer i;
    initial begin
		  // this should actually be (i < (2 ** ADDRESS_WIDTH)) but the loop counter doesn't got that high...
        for (i = 0; i < DATA_WIDTH; i = i + 1) begin 
            mem_data[i] = 0;
        end     
    end
    
	assign data_out = mem_data[Address];

    always @ (posedge Clk) begin
        if (MemW) 
            mem_data[Address] <= data_in;
    end
endmodule				  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	

module Stack(
  input clk,
  input pushorpop,
  input [31:0] data_in,
  input [5:0] opcode,
  input [25:0] imme,
  output reg [31:0] data_out 
);

  reg [31:0] stack [0:31];
  reg [4:0] top = 5'b00000;

  always @(posedge clk) begin	
	  
	if (pushorpop == 2'b11) begin 	// PUSH
      if (top < 31) begin
        stack[top] = data_in;
		top = top + 5'b00001;
      end
    end else if (pushorpop == 2'b01) begin   // POP or  RET
      if (top > 5'b00000) begin	 
		top = top - 5'b00001;
        data_out = stack[top]; 
      end
    end	 	
  else if (pushorpop == 2'b00) begin // CALL	 
	  	  // PC + 1 is pushed onto the stack  (data_in)
    if (top < 31) begin
        stack[top] = data_in;
        top = top + 5'b00001;
        // Next PC is set to the address formed by concatenating PC[31:26] with Immediate26
        data_out = {opcode, imme};
    end  
  end 
	
  end

endmodule  		 

module Stack_tb;

    // Inputs
    reg clk;
    reg [1:0] pushorpop;
    reg [31:0] data_in;
    reg [5:0] opcode;
    reg [25:0] imme;

    // Outputs
    wire [31:0] data_out;

    // Instantiate the Unit Under Test (UUT)
    Stack uut (
        .clk(clk),
        .pushorpop(pushorpop),
        .data_in(data_in),
        .opcode(opcode),
        .imme(imme),
        .data_out(data_out)
    );

    initial begin
        // Initialize inputs
        clk = 0;
        pushorpop = 2'b00; // Initialize for CALL
        data_in = 32'h12345678; // Example data to be pushed
        opcode = 6'b011111; // Example opcode
        imme = 26'hABCDEF; // Example immediate value

        // Apply stimulus for CALL
        #10; // Wait for a few clock cycles
        pushorpop = 2'b11; // Change to PUSH
        #10; // Wait for a few clock cycles
        pushorpop = 2'b01; // Change to POP
        #10; // Wait for a few clock cycles

        // Finish the simulation
        $finish;
    end

    // Clock generation
    always begin
        #5 clk = ~clk;
    end

    // Display the stack content when a PUSH operation occurs
    always @(posedge clk) begin
        if (pushorpop == 2'b11) begin
            $display("PUSH: Data pushed onto the stack = %h", data_in);
        end
    end

endmodule


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
module mux2x1(select,I0, I1, out);
	parameter n = 4;
	input select;
	input[(n-1):0] I0, I1;
	output[(n-1):0] out;
	
	assign out = (select) ? I1: I0;
	
endmodule	

module mux2x1_tb;

    // Inputs
    reg select;
    reg [3:0] I0, I1;

    // Output
    wire [3:0] out;

    // Instantiate the Unit Under Test (UUT)
    mux2x1 uut (
        .select(select), 
        .I0(I0), 
        .I1(I1), 
        .out(out)
    );

    initial begin
        // Initialize Inputs
        select = 0;
        I0 = 4'b0000;
        I1 = 4'b1111;

        // Apply stimulus
        select = 1'b0;
        #30; // Wait for 10 time units

        select = 1'b1;
        #30; // Wait for 10 time units

        // Finish the simulation
        $finish;
    end
      
endmodule


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
module mux4x1(select,I0, I1, I2, I3, out);
	parameter n = 32;
	input[1:0] select;
	input[(n-1):0] I0, I1, I2, I3;
	output[(n-1):0] out;
	
	assign out = (select[1]) ? ((select[0])? I3: I2):((select[0])? I1: I0);
	
endmodule	


module mux4x1_tb;

    parameter n = 32;

    // Inputs
    reg [1:0] select;
    reg [(n-1):0] I0, I1, I2, I3;

    // Output
    wire [(n-1):0] out;

    // Instantiate the Unit Under Test (UUT)
    mux4x1 #(.n(n)) uut (
        .select(select), 
        .I0(I0), 
        .I1(I1), 
        .I2(I2), 
        .I3(I3), 
        .out(out)
    );

    initial begin
        // Initialize Inputs
        select = 0;
        I0 = 0;
        I1 = 0;
        I2 = 0;
        I3 = 0;

        // Wait for 100 ns for global reset
        #100;
        
        // Add stimulus here
        I0 = 32'hAAAAAAAA; // Example pattern
        I1 = 32'h55555555; // Example pattern
        I2 = 32'h33333333; // Example pattern
        I3 = 32'h77777777; // Example pattern

        select = 2'b00;
        #20; // Wait for 10 ns

        select = 2'b01;
        #20; // Wait for 10 ns

        select = 2'b10;
        #20; // Wait for 10 ns

        select = 2'b11;
        #20; // Wait for 10 ns

        // Finish the simulation
        $finish;
    end
      
endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
module alu (
    input [31:0] in1,
    input [31:0] in2,
    input [1:0] ALUop,
    output reg [31:0] out1,
    output reg [1:0] zeroflag // Now a 2-bit register
);

// Define the opcodes for different ALU operations
parameter ADD = 2'b00;
parameter AND = 2'b01;
parameter CMP = 2'b10; 
parameter SUB = 2'b11;
// Add other operation codes as needed...

always @(*) begin
    // Perform the operation based on the opcode
    case (ALUop)
        ADD: out1 = in1 + in2;
        AND: out1 = in1 & in2;	
		SUB: out1 = in1 - in2;
        CMP: begin
            // For CMP operation, we only care about setting the zeroflag
            out1 = 32'b0; // out1 doesn't need to hold a meaningful result
            if (in1 == in2) begin
                zeroflag = 2'b00; // Equal
            end else if ($signed(in1) > $signed(in2)) begin
                zeroflag = 2'b01; // in1 greater than in2
            end else begin
                zeroflag = 2'b10; // in1 less than in2
            end	
			
        end
        // Add other operations as needed...
        default: begin
            out1 = 32'b0; // Default case to handle undefined opcodes
            zeroflag = 2'b00; // Default case for zeroflag
        end
    endcase
end

endmodule	 

module alu_tb;

    // Inputs
    reg [31:0] in1, in2;
    reg [1:0] ALUop;

    // Outputs
    wire [31:0] out1;
    wire [1:0] zeroflag;

    // Instantiate the ALU module
    alu uut (
        .in1(in1),
        .in2(in2),
        .ALUop(ALUop),
        .out1(out1),
        .zeroflag(zeroflag)
    );

    // Clock generation
    reg clk = 0;
    always #5 clk = ~clk;

    // Initial block to apply stimulus
    initial begin
        // Test case 1: ADD operation
        in1 = 8'h0A;
        in2 = 8'h02;
        ALUop = 2'b00;

        #10; // Wait for some time

        // Test case 2: AND operation
        in1 = 8'h0F;
        in2 = 8'h04;
        ALUop = 2'b01;

        #10; // Wait for some time

        // Test case 3: SUB operation
        in1 = 8'h0A;
        in2 = 8'h04;
        ALUop = 2'b11;

        #10; // Wait for some time

        // Test case 4: CMP operation
        in1 = 8'h0A;
        in2 = 8'h0A;
        ALUop = 2'b10;

        #10; // Wait for some time

        // Add more test cases as needed...

        $stop; // Stop the simulation
    end

    // Monitor to display outputs
    always @(posedge clk) begin
        $display("in1 = %h, in2 = %h, ALUop = %h, out1 = %h, zeroflag = %h",
                 in1, in2, ALUop, out1, zeroflag);
    end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
module control_signals(
    input clk,
    input rst,
    input [5:0] Opcode,
    output reg PCWrite,
    output reg IRWrite,     
    output reg [1:0] ALUOp,
    output reg MemWR, 
    output reg MemRD, 
    output reg [1:0] MemToReg,      
    output reg RegWr, 
    output reg ExtOp,       
    output reg [1:0] PCsrc,
    output reg [1:0] ALUSrc,    
    output reg RWSrc, 
    output reg RBSrc, 
    output reg StackInSrc,
    output reg [1:0] BranchType, 
	output reg [1:0] pushorpop
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        // Reset all control signals to default values
        PCWrite <= 0;
        IRWrite <= 0;
        ALUOp <= 0;
        MemWR <= 0;
        MemRD <= 0;
        MemToReg <= 0;
        RegWr <= 0;
        ExtOp <= 0;
        PCsrc <= 0;
        ALUSrc <= 0;
        RWSrc <= 0;
        RBSrc <= 0;
        StackInSrc <= 0;
        BranchType <= 0; 
		pushorpop <= 0;
    end else begin
        // Default values for control signals
        PCWrite <= 0;
        IRWrite <= 0;
        ALUOp <= 0;
        MemWR <= 0;
        MemRD <= 0;
        MemToReg <= 0;
        RegWr <= 0;
        ExtOp <= 0;
        PCsrc <= 0;
        ALUSrc <= 0;
        RWSrc <= 0;
        RBSrc <= 0;
        StackInSrc <= 0;
        BranchType <= 0;

        // Determine the control signals based on Opcode
        if (Opcode == 6'b000000) begin
            ALUOp <= 2'b00; // AND operation
			MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 0;
			RegWr <= 1;	
			//ExtOp <= 0;  
			ALUSrc <= 2'b00;
			MemWR <= 0;	
			MemToReg <= 2'b00;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			//StackInSrc <= 0;
        	//BranchType <= 0;  
        end else if (Opcode == 6'b000001) begin
            ALUOp <= 2'b01; // ADD operation  
			MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 0;
			RegWr <= 1;	
			//ExtOp <= 0;  
			ALUSrc <= 2'b00;
			MemWR <= 0;	
			MemToReg <= 2'b00;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			//StackInSrc <= 0;
        	//BranchType <= 0;  
        end else if (Opcode == 6'b000010) begin
            ALUOp <= 2'b11; // SUB operation  
			MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 0;
			RegWr <= 1;	
			//ExtOp <= 0;  
			ALUSrc <= 2'b00;
			MemWR <= 0;	
			MemToReg <= 2'b00;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			//StackInSrc <= 0;
        	//BranchType <= 0;  
        end	else if (Opcode == 6'b000011) begin
            ALUOp <= 2'b00; // AND operation  
			MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 0;
			RegWr <= 1;	
			ExtOp <= 0;  
			ALUSrc <= 2'b01;
			MemWR <= 0;	
			MemToReg <= 2'b00;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			//StackInSrc <= 0;
        	//BranchType <= 0;  
        end else if (Opcode == 6'b000100) begin
            ALUOp <= 2'b01; // ADD operation  
			MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 0;
			RegWr <= 1;	
			ExtOp <= 1;  
			ALUSrc <= 2'b01;
			MemWR <= 0;	
			MemToReg <= 2'b00;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			//StackInSrc <= 0;
        	//BranchType <= 0;  
        end else if (Opcode == 6'b000101 || Opcode == 6'b000110) begin
            ALUOp <= 2'b01; // ADD operation  
			MemRD <= 1;
			RWSrc <= 0;
        	RBSrc <= 0;
			RegWr <= 1;	
			ExtOp <= 1;  
			ALUSrc <= 2'b01;
			MemWR <= 0;	
			MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			//StackInSrc <= 0;
        	//BranchType <= 0;  
        end	else if (Opcode == 6'b000111) begin
            ALUOp <= 2'b01; // ADD operation  
			MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 1;
			RegWr <= 0;	
			ExtOp <= 1;  
			ALUSrc <= 2'b01;
			MemWR <= 1;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			//StackInSrc <= 0;
        	//BranchType <= 2'00;  
        end	else if (Opcode == 6'b001000) begin
            ALUOp <= 2'b10; // CMP operation  
			//MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 1;
			RegWr <= 0;	
			ExtOp <= 1;  
			ALUSrc <= 2'b00;
			MemWR <= 0;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b01;
			//StackInSrc <= 0;
        	BranchType <= 2'b00;  
        end else if (Opcode == 6'b001001) begin
            ALUOp <= 2'b10; // CMP operation  
			//MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 1;
			RegWr <= 0;	
			ExtOp <= 1;  
			ALUSrc <= 2'b00;
			MemWR <= 0;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b01;
			//StackInSrc <= 0;
        	BranchType <= 2'b01;  
        end	else if (Opcode == 6'b001010) begin
            ALUOp <= 2'b10; // CMP operation  
			//MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 1;
			RegWr <= 0;	
			ExtOp <= 1;  
			ALUSrc <= 2'b00;
			MemWR <= 0;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b01;
			//StackInSrc <= 0;
        	BranchType <= 2'b10;  
        end else if (Opcode == 6'b001011) begin
            ALUOp <= 2'b10; // CMP operation  
			//MemRD <= 0;
			RWSrc <= 0;
        	RBSrc <= 1;
			RegWr <= 0;	
			ExtOp <= 1;  
			ALUSrc <= 2'b00;
			MemWR <= 0;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b01;
			//StackInSrc <= 0;
        	BranchType <= 2'b11;  
        end	 else if (Opcode == 6'b001100) begin  //JMP
          //  ALUOp <= 2'b10; // CMP operation  
			MemRD <= 0;
			//RWSrc <= 0;
        	//RBSrc <= 1;
			RegWr <= 0;	
			//ExtOp <= 1;  
			//ALUSrc <= 2'b00;
			MemWR <= 0;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b00;
			//StackInSrc <= 0;
        	//BranchType <= 2'11;  
        end else if (Opcode == 6'b001101 ) begin
          //  ALUOp <= 2'b10; // CMP operation  
			MemRD <= 0;
			RWSrc <= 0;
        	//RBSrc <= 1;
			RegWr <= 0;	
			//ExtOp <= 1;  
			//ALUSrc <= 2'b00;
			MemWR <= 0;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b10;
			StackInSrc <= 0;
        	//BranchType <= 2'b11; 
			pushorpop <= 2'b00;
        end	else if (Opcode == 6'b001110) begin
          //  ALUOp <= 2'b10; // CMP operation  
			MemRD <= 0;
			RWSrc <= 0;
        	//RBSrc <= 1;
			RegWr <= 0;	
			//ExtOp <= 1;  
			//ALUSrc <= 2'b00;
			MemWR <= 0;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b10;
			StackInSrc <= 0;
        	//BranchType <= 2'b11; 
			pushorpop <= 2'b01;
        end
		else if (Opcode == 6'b001111) begin
          //  ALUOp <= 2'b10; // CMP operation  
			MemRD <= 0;
			RWSrc <= 0;
        	//RBSrc <= 1;
			RegWr <= 0;	
			//ExtOp <= 1;  
			//ALUSrc <= 2'b00;
			MemWR <= 0;	
			//MemToReg <= 2'b01;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			StackInSrc <= 1;
        	//BranchType <= 2'b11;  
			pushorpop <= 2'b11	;
        end	else if (Opcode == 6'b010000) begin
          //  ALUOp <= 2'b10; // CMP operation  
			MemRD <= 0;
			RWSrc <= 0;
        	//RBSrc <= 1;
			RegWr <= 1;	
			//ExtOp <= 1;  
			//ALUSrc <= 2'b00;
			MemWR <= 0;	
			MemToReg <= 2'b10;	
			PCWrite <= 1;  
			PCsrc <= 2'b11;
			StackInSrc <= 1;
        	//BranchType <= 2'11;  
			pushorpop <= 2'b01;
        end	
 
    end
end

endmodule	

module control_signals_tb;

    // Inputs
    reg clk;
    reg rst;
    reg [5:0] Opcode;

    // Outputs
    wire PCWrite;
    wire IRWrite;
    wire [1:0] ALUOp;
    wire MemWR;
    wire MemRD;
    wire [1:0] MemToReg;
    wire RegWr;
    wire ExtOp;
    wire [1:0] PCsrc;
    wire [1:0] ALUSrc;
    wire RWSrc;
    wire RBSrc;
    wire StackInSrc;
    wire [1:0] BranchType;
    wire [1:0] pushorpop;

    // Instantiate the control_signals module
    control_signals uut (
        .clk(clk),
        .rst(rst),
        .Opcode(Opcode),
        .PCWrite(PCWrite),
        .IRWrite(IRWrite),
        .ALUOp(ALUOp),
        .MemWR(MemWR),
        .MemRD(MemRD),
        .MemToReg(MemToReg),
        .RegWr(RegWr),
        .ExtOp(ExtOp),
        .PCsrc(PCsrc),
        .ALUSrc(ALUSrc),
        .RWSrc(RWSrc),
        .RBSrc(RBSrc),
        .StackInSrc(StackInSrc),
        .BranchType(BranchType),
        .pushorpop(pushorpop)
    );

    // Initialize signals
    initial begin
        clk = 0;
        rst = 0;
        Opcode = 6'b000000; // Default Opcode

        // Stimulus
        #10 rst = 1; // Assert reset
        #10 rst = 0; // Deassert reset

        // Test cases
        #20 Opcode = 6'b000010; // Test case 1: Opcode 000010
        #30 Opcode = 6'b001000; // Test case 2: Opcode 001000
        // Add more test cases as needed...

        $finish; // End simulation
    end

    // Clock generation
    always #5 clk = ~clk;

endmodule



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
module extender(
    input extendLorA,
    input [15:0] imm,
    output reg [31:0] out
);

always @(*) begin
    if (extendLorA == 1'b0) begin
        // Zero extend
        out = {16'b0, imm};
    end else begin
        // Sign extend
        out = {{16{imm[15]}}, imm};
    end
end

endmodule	 


module extender_tb;

    // Inputs
    reg extendLorA;
    reg [15:0] imm;

    // Outputs
    wire [31:0] out;

    // Instantiate the extender module
    extender uut (
        .extendLorA(extendLorA),
        .imm(imm),
        .out(out)
    );

    // Test cases
    initial begin
        // Test case 1: Sign extend
        extendLorA = 1;
        imm = 16'hFF00;
        #10; // Wait for output to stabilize
        // Verify the output here

        // Test case 2: Zero extend
        extendLorA = 0;
        imm = 16'hFF00;
        #10; // Wait for output to stabilize
 
        $finish;
    end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	

module adder_32bit(
    input [31:0] in1,
    input [31:0] in2,
    output [31:0] sum
);

assign sum = in1 + in2;

endmodule 

module adder_32bit_tb;

    // Inputs
    reg [31:0] in1, in2;

    // Outputs
    wire [31:0] sum;

    // Instantiate the adder_32bit module
    adder_32bit uut (
        .in1(in1),
        .in2(in2),
        .sum(sum)
    );

    // Test cases
    initial begin
        // Test case 1: Positive numbers
        in1 = 32'h00000001;
        in2 = 32'h00000002;
        #10; // Wait for some time to allow the addition to propagate
        // Verify the sum here

        // Test case 2: Negative numbers
        in1 = 32'hFFFFFFFE;
        in2 = 32'hFFFFFFFF;
        #10;
        // Verify the sum here

        // Test case 3: Large numbers
        in1 = 32'h7FFFFFFF;
        in2 = 32'h80000000;
        #10;
        // Verify the sum here

        // Add more test cases as needed...

        $finish;
    end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
module branch_controller(
    input [1:0] zeroFlag,
    input [31:0] imm,    
    input [31:0] pc,
    input [1:0] branchType,  
    output reg [31:0] nextPC
);  
    
    parameter BGT = 2'b00;
    parameter BLT = 2'b01;
    parameter BEQ = 2'b10; 
    parameter BNE = 2'b11;
    
    
  
    always @(*) begin
        // Default next PC to increment by 1 (sequential execution)
        nextPC = pc + 32'd1;
        
        // Check branchType and zeroFlag to determine the correct next PC
        case (branchType)
            BGT: begin
                if (zeroFlag == 2'b10) // Greater than
                    nextPC = pc + imm;
            end
            BLT: begin
                if (zeroFlag == 2'b01) // Less than
                    nextPC = pc + imm;
            end
            BEQ: begin
                if (zeroFlag == 2'b00) // Equal to
                    nextPC = pc + imm;
            end
            BNE: begin
                if (zeroFlag != 2'b00) // Not equal to
                    nextPC = pc + imm;
            end
            default: begin
                // For other cases, the default next PC will be used (pc + 1)
            end
        endcase
    end
endmodule  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 	
module multicycle();   
	reg clk;  
	reg rst; 
	wire [31:0] nextPC ;	  
	wire [31:0] currentPC ;	
	wire [3:0] RA ;
	wire [3:0] RB ;  
	wire [3:0] RW ;
	wire PCWrite;
	wire [5:0] Opcode;
    wire IRWrite;     
    wire [1:0] ALUOp;
    wire MemWR; 
    wire MemRD; 
    wire [1:0] MemToReg;      
    wire RegWr; 
    wire ExtOp;       
    wire [1:0] PCsrc;
    wire [1:0] ALUSrc;    
    wire RWSrc; 
    wire RBSrc; 
    wire StackInSrc;
    wire [1:0] BranchType; 
	wire [1:0] pushorpop ; 
	wire [15:0] Imm;   
	wire [1:0] mode; 
	wire[25:0] Jump_Imm;  
	wire [31:0] data_out;
	
	wire [31:0] PCplusOne;
	
	wire [3:0] finalRB ;
	wire [3:0] finalRW ;  
	
	wire [31:0] dataToBeWrittenInRD;   
	
	wire [31:0] BusA, BusB;	
	wire [31:0] immediateExtended; 
	wire [31:0] ALUIn2;	  
	wire [31:0] ALUOutput;	 
	wire  [1:0] flag;  
	wire  [31:0] datamemoryOutput;
	wire [31:0] jumpReg; 
	wire [31:0] branchReg; 	 
	wire [31:0] stackinput; 
	wire [31:0] stackReg; 
	
	
	reg_pc pc(
		.clk(clk),
		.rst(rst),
		.newPC(nextPC),
		.PCWrite(PCWrite),
		.PC(currentPC)
    );	
		
	InstructionMemory IM(.data_out(data_out), .address(currentPC));
	
     reg_ir ir(
		.clk(clk),
		.rst(rst),
		.IRWrite(IRWrite),
		.InstrIn(data_out),
		.Opcode(Opcode),
		.RS1(RA),
		.RS2(RB),
		.RW(RW),
		.Imm(Imm),	
		.Mode(mode),
		.Jump_Imm(Jump_Imm)
    );	
	
	
 control_signals CS(
    .clk(clk),
    .rst(rst),
    .Opcode(Opcode),
    .PCWrite(PCWrite),
   .IRWrite(IRWrite),     
    .ALUOp(ALUOp),
    .MemWR(MemWR), 
    .MemRD(MemRD), 
    .MemToReg(MemToReg),      
    .RegWr(RegWr), 
    .ExtOp(ExtOp),       
    .PCsrc(PCsrc),
    .ALUSrc(ALUSrc),    
    .RWSrc(RWSrc), 
    .RBSrc(RBSrc), 
    .StackInSrc(StackInSrc),
    .BranchType(BranchType), 
	.pushorpop(pushorpop)
);	

 
extender wx(
    .extendLorA(ExtOp),
    .imm(Imm),
    .out(immediateExtended)
);

mux2x1 muxRB(RBSrc,RB, RW, finalRB);
mux2x1 muxRW(RWSrc,RW, RA, finalRW);   		 

RegisterFile  RF(
	.rs1(RA), .rs2(finalRB), .rd(finalRW),
	.clk(clk), .writeSignal(RegWr),
	.data(dataToBeWrittenInRD),
	.dataOut1(BusA), .dataOut2(BusB)
    );
	
mux4x1  muxALU2(ALUSrc,BusB, immediateExtended, 32'b1, 32'b0, ALUIn2 );

alu alu(
    .in1(BusA),
    .in2(ALUIn2),
    .ALUop(ALUOp),
    .out1(ALUOutput),
    .zeroflag(flag) 
);

DataMemory DM( .data_out(datamemoryOutput),     
	  		.data_in(BusB),   
            .Address(ALUOutput),     
            .MemW(MemWR),      
			.MemRd(MemRD),
            .Clk(clk));       
    
adder_32bit Adder(
    .in1(currentPC),
    .in2(32'b100),
    .sum(PCplusOne)
); 

adder_32bit Adder1(
    .in1(currentPC),
    .in2(immediateExtended),
    .sum(jumpReg)
);

branch_controller bc(
    .zeroFlag(flag),
    .imm(immediateExtended),    
    .pc(currentPC),
    .branchType(BranchType),  
    .nextPC(branchReg)
);   

mux2x1 muxStack(StackInSrc,PCplusOne, immediateExtended, stackinput);


Stack  st(
  .clk(clk),
  .pushorpop(pushorpop),
  .data_in(stackinput),
  .opcode(Opcode),
  .imme(Jump_Imm),
  .data_out(stackReg) 
);  
	
mux4x1 muxWriteBack(MemToReg,ALUOutput, datamemoryOutput, stackReg, 0,dataToBeWrittenInRD );	

mux4x1 pcNext(PCsrc,jumpReg, branchReg, stackReg, PCplusOne, nextPC);
	
initial begin
	clk = 0; 
	
	repeat(200)
	#5ns clk = !clk;
	$finish;
end

	
endmodule 

