`define  IF 3'b000  //Fetch 
`define  ID 3'b001	//Decode  
`define  EX 3'b010	//Execute
`define  MEM 3'b011	//Memory
`define  WB 3'b100	//WriteBack	
///////Define the ALU operation codes
`define  AND 2'b00  //AND Operation 
`define  ADD 2'b01	//ADD Operatoion  
`define  SUB 2'b10	//SUB Operation

///Multi-Cycle Processor
module MultiCycleProcessor (clk, reset, zeroFlag, overflowFlag, negativeFlag, ALUOut, dataOut, WBData);  
	// input to our processor (just we need clock and reset to reintiate the values)
	input clk, reset;
	
	// output from our processor which used for waveform
	output reg zeroFlag, overflowFlag, negativeFlag; // output flags from ALU (which needed to check branch taken or not taken)
	output reg signed [31:0] ALUOut;// ALU output
	
	// all needed signals for main control unit
	reg Reg2Src, RegW, Rs1W, ALUsrc, ExtOp, DataSrc, AddSrc, MemR, MemW, DataWB;
	
	// signal for ALU control unit
	reg [1:0]ALUOp; // two signals for define the ALU operation --> +, - or &
	
	// define the Instruction Memory
	// the actual Instruction Memory
	reg [31:0] Memory[0:1023]; // 32-bit address memory, its size = 1024 byte and it can be more or less (as computer spec)
	
	// define the data and stack memroy 
	// the actual Data & Stack Memory
	reg signed [31:0] Data_SatckMemory[0:1023]; // 32-bit address memory, also this size is temporary and can be changed
	
	//define the special purpose registers --> PC & SP
	reg [31:0] PC;
	reg [31:0] NPC;// save next PC used for call instructon this is used for CALL instruction (variable to save the next value of PC because the original value will be updated)
	reg [31:0] SP; // stack pointer register pointing at top of the stack
		
	reg [2:0] PState, NState;// states used in FSM
	
	
	
	reg [31:0] IR; // instruction register used to save the readed instruction obtained from instruction memory

	
	/////Define the addresses of the registers being used in Register File
	reg [3:0] Rd;    //Destination address
	reg [3:0] Rs1;	 //Source1 address
	reg [3:0] Rs2;	 //Source2 address	
	
	// two bit mode for I-type instructions
	reg [1:0] Mode;
	
	reg signed [31:0]extendedImm; // the output for EXtender 
	
	// Define the Register File
	reg signed [31:0] Registers[0:15]; // 16 32-bit general purpose registers 
	reg signed [31:0] BusA, BusB;// output form Register File
	reg signed [31:0] BusW, BusWA;// Write buses for Register File
		
	// variable to save address used in memory, data in and data out from memory	
	reg signed [31:0] Memaddress;
	reg signed [31:0] dataIn;
	output reg signed [31:0] dataOut;
	output reg signed [31:0] WBData; // Data be written to Rigster file (last MUX)
	

	integer i; // used to reinitiate the values saved in registers when there is reset input
	initial 
	begin
		PC = 32'b0; // the first address for PC starting from address zero 
		NState = `IF; // the first cycle for program is fetch of course 
		SP = 32'd768; // let the stack have 256 last cells of memory (can be changed as computer spec)
		for (i = 0; i <= 15; i = i + 1) // reset all values for registers 
	 		Registers[i] = 32'h00000000;
		
		$display("\n****************Welcome to our processor****************\n");
		
			// program instrution Memory
			Memory[0] = 32'b00010100100000000000000000000000; // LW R2, 0(R0) --> R0 points to the address of data being loadded (R2 = 2)
			Memory[1] = 32'b00010000000000000000000000001100; // ADDI R0, R0, 3
			Memory[2] = 32'b00010100010000000000000000000000; // LW R1, 0(R0) --> R0 points to the address of data being loadded (R1 = 1)
			Memory[3] = 32'b00000000110001001000000000000000; // AND R3, R1, R2
			Memory[4] = 32'b00000100110001001000000000000000; // ADD R3, R1, R2
			Memory[5] = 32'b00010101000101000000000000000100; // LW R4, 1(R5), R4 = 20
			Memory[6] = 32'b00010101100101000000000000001001; // LW.POI R6, 2(R5), R6 = 23
			Memory[7] = 32'b00101001000110000000000000001100; // loop: BEQ R4, R6, toPush --> Memory[10]
			Memory[8] = 32'b00001001100110000100000000000000; // SUB R6, R6, R1 --> R6-=1
			Memory[9] = 32'b00110000000000000000000000000111; // JMP loop --> to Memory[7]
			Memory[10] = 32'b00111101100000000000000000000000; //// toPush: PUSH R6
			Memory[11] = 32'b00010101110000000000000000001000; // LW R7, 2(R0) --> R0 points to the address of data being loadded (R2 = 2)
			Memory[12] = 32'b00001100000111000000000000000100; // ANDI R0, R7, 1
			Memory[13] = 32'b00101100000010000000000000010000; //  BNE R7, R2, store  Memory[17]
			Memory[14] = 32'b00110000000000000000000000010011; // JMP Func2 --> to Memory[19]
			Memory[15] = 32'b00000100000001001000000000000000; /// Func: ADD R0, R1, R2
			Memory[16] = 32'b00111000000000000000000000000000; /// RET
			Memory[17] = 32'b00011100000011000000000000001000; // store: SW R0, 2(R3)
			Memory[18] = 32'b00110100000000000000000000001111; /// CALL the Func	 --> to Memory[15]
			Memory[19] = 32'b00011100000011000000000000001000; // Func2: SW R0, 2(R3) ---> store in Data_SatckMemory[4]
			Memory[20] = 32'b01000000000000000000000000000000; ////  POP R0
			///Data
			Data_SatckMemory[0] = 32'b00000000000000000000000000000010; /// 2 loaded by R2
			Data_SatckMemory[1] = 32'b00000000000000000000000000010100; /// 20 in decimal 
			Data_SatckMemory[2] = 32'b00000000000000000000000000010111;	//// 23 in decimal
			Data_SatckMemory[3] = 32'b00000000000000000000000000000001;	//// 1 in decimal
			// No data in address 4
			Data_SatckMemory[5] = 32'b01111111111111111111111111111111;	//// big value to check overflow
	end
	
	
	
	always @(posedge clk, posedge reset) begin
		
		if (reset)	
			begin //empty all register values and make state to IF stage  
				$display("\n------------------Reset Program------------------\n");
				NState = `IF; ///Initialize the next state
				PC = 32'h00000000;
				Rs1 = 32'd0;
				Rs2 = 32'd0;
				Rd = 32'd0;				
				
				for (i = 0; i <= 15; i = i + 1)
	 			 	Registers[i] = 32'h00000000;
			end	
		else
			begin
				PState = NState;
				
				case(PState)
	//------------------------------------IF-----------------------------------------
					`IF: 
						begin
							$display("\n------------------Fetch Cycle------------------\n");
							$display("PC=%d\t", PC);
			  				InstructionMemory(PC,IR); // get instruction from instruction memory
			  				NState=`ID; // change the state instruction
							  
							
						end
						
	//------------------------------------ID-----------------------------------------
					`ID:
						begin
						  	$display("\n------------------Decode Cycle------------------\n");
							// Generate Main Control signals
							MainControl(IR[31:26]);
							$display("Signals:\nReg2Src = %b, RegW = %b, Rs1W, = %b, ALUsrc = %b, ExtOp = %b, DataSrc = %b, AddSrc = %b, MemR = %b, MemW = %b, DataWB = %b\n",Reg2Src, RegW, Rs1W, ALUsrc, ExtOp, DataSrc, AddSrc, MemR, MemW, DataWB);
				
							Rd = IR[25:22];
							Rs1 = IR[21:18];
							if(Reg2Src)
								Rs2 = IR[17:14];
							else
								Rs2 = Rd;
								
							
								
							// Extend Imm	
							Extender(IR[17:2], ExtOp, extendedImm);
							
							// Read from Register File
							RegsiterFileRead(Rs1,Rs2, BusA, BusB);
							
								
							// ******* Instruction dont need to go to execute stage (J-type & S-type)
							if(IR[31:26] == 6'b001100) // JMP instruction need 2 cycles
								begin
									PC = {PC[31:26], IR[25:0]}; //JTA then to fetch stage
									NState=`IF;	
								end
							else if(IR[31:26] == 6'b001101) // CALL instruction need to go to Memory stage
								begin
									NPC = PC + 1;
									PC = {PC[31:26], IR[25:0]};
									NState=`MEM;	
								end
							else if(IR[31:26] == 6'b001110 || IR[31:26] == 6'b001111 || IR[31:26]==6'b010000) // RET, PUSH and POP instructions need to go to Memory stage
								NState=`MEM;
							else // other instrucations need to go to Execute stage
								NState = `EX;		
						end
	//------------------------------------EX-----------------------------------------			
					`EX:
						begin			
							$display("\n------------------Execute Cycle------------------\n");
							$display("Extended immediate=%2d, BusA=%2d, BusB=%2d\n", extendedImm, BusA, BusB);
							if (ALUsrc == 1'b0)
								ALU(ALUOp, BusA, BusB, ALUOut, zeroFlag, negativeFlag, overflowFlag);
							else
								ALU(ALUOp, BusA, extendedImm, ALUOut, zeroFlag, negativeFlag, overflowFlag);
							
							
							if (IR[31:26] <= 6'b000100) // R-Type, ADDI, ANDI instructions
								NState = `WB;
							else if (IR[31:26] == 6'b000101 || IR[31:26] == 6'b000111) // LW, SW instructions
								NState = `MEM;
							else if (IR[31:26] >= 6'b001000 && IR[31:26] <= 6'b001011) // Branch instructions
								NState = `IF;
							
							// check conditoins for branch instructions
							// BEQ instruction
							if (IR[31:26] == 6'b001010) 
								begin
									if (zeroFlag == 1'b1) 
										PC = PC + extendedImm; //branch taken --> PC = BTA
									else
										PC = PC + 1;
								end	  
							
							// BNE instruciton
							else if (IR[31:26] == 6'b001011) 
								begin
									if (zeroFlag == 1'b0) 
										PC = PC + extendedImm; //branch taken --> PC = BTA
									else
										PC = PC + 1; //branch not taken --> PC = PC + 1
								end
							
							// BGT instruction
							else if (IR[31:26] == 6'b001000) 
								begin
									if (negativeFlag != overflowFlag)
										PC = PC + extendedImm; //branch taken --> PC = BTA
									else
										PC = PC + 1;
								end
							
							// BLT instruction
							else if (IR[31:26] == 6'b001001) 
								begin
									if ((negativeFlag == overflowFlag) && !zeroFlag) // Rd < Rs1 same as --> Rs1 > Rd because the first operand in ALU remain Rs1 
										PC = PC + extendedImm;							   
									else
										PC = PC + 1;
								end 
						end // end EX
	//------------------------------------MEM-----------------------------------------				
					`MEM: 
						begin
							$display("\n------------------Memory Cycle------------------\n");
							
							if(AddSrc) // when the address is the SP
								begin
									if(IR[31:26] == 6'b001101 || IR[31:26] == 6'b001111)
										begin // CALL and PUSH instructions
											Memaddress = SP;
											SP = SP + 1;
										end
									else if(IR[31:26] == 6'b001110 || IR[31:26] == 6'b010000) // RET & POP instructions
										begin
											Memaddress = SP - 1;
											SP = SP - 1;
										end
								end
							else
								Memaddress = ALUOut;
								
							if(DataSrc)
								dataIn = BusB;
							else
								dataIn = NPC;
								
							dataMem(Memaddress, dataIn, MemR, MemW, dataOut);
							
							if(IR[31:26] == 6'b001101) // CALL instructin next state --> Fetch
								NState = `IF;
							else if(IR[31:26] == 6'b001110) // RET instructin next state --> Fetch 
								begin
									PC = dataOut;
									NState = `IF;
								end
						
							else if(IR[31:26] == 6'b000111 || IR[31:26] == 6'b001111) // SW and PUSH need to go to fetch
								begin
									PC = PC + 1;
									NState = `IF;
								end
							else //LW, LW.POI and POP --> WB
								NState = `WB;			
						end
	//------------------------------------WB-----------------------------------------
				
					`WB: 
						begin
							$display("\n------------------Write Back Cycle------------------\n");
							
							if(DataWB==1)
								WBData = ALUOut;
							else
								WBData = dataOut;
							
							RegsiterFileWrite(Rd, RegW, Rs1W, WBData, BusA + 1);
							PC = PC + 1;
							NState = `IF;
																			  
								
						end	// end WB 
				endcase
			end // end else
		end // end always
	


// -------------------- push --------------------------
task PushOnStack(input [31:0] value);
	Data_SatckMemory[SP] = value; 
	$display("Push in the Stack value = %b\n", value);
	SP = SP + 1;
endtask

// -------------------- pop --------------------------
task PopFromStack(output reg [31:0] pop_value);
	SP = SP - 1;
	pop_value = Data_SatckMemory[SP];
	$display("Pop from the Stack value = %b\n", pop_value);
endtask


// -------------------- InstructionMemory --------------------------
task InstructionMemory(input [31:0] instAdd, output reg [31:0] instOut);
	
	instOut = Memory[instAdd];
	
	$display("Instruction = %b\n", instOut);
endtask


// -------------------- Data Memory --------------------------	
task dataMem(input [31:0] Address, input signed [31:0] Data_in, input MemR, MemW, output signed [31:0] Data_out);
	if(MemR==2'b1 && MemW==2'b0)
		Data_out=Data_SatckMemory[Address];	
	else if(MemR==2'b0 && MemW==2'b1)
		Data_SatckMemory[Address]=Data_in;
		
	if(MemR == 1 || MemW == 1)
		begin
			$display("Data_in: %2d, Data_out: %2d, Address: %2d", Data_in, Data_out, Address);
			$display("MemR: %b, MemW: %b, Data_SatckMemory[%0d]: %2d\n", MemR, MemW, Address,Data_SatckMemory[Address]);
		end
endtask


// -------------------- Register Read --------------------------
task RegsiterFileRead(input [3:0] Rs1, Rs2, output signed [31:0] BusA, BusB);
	BusA = Registers[Rs1];
	BusB = Registers[Rs2];
endtask

// -------------------- Register Write --------------------------
task RegsiterFileWrite(input [3:0] Rd,  input RegW, input Rs1W, input [31:0] BusW, input [31:0] BusWA);
	if(Rs1W)
		Registers[Rs1] = BusWA;
	if(RegW)
		Registers[Rd] = BusW;
	if(Rs1W)
		$display("The register R%0d updated with new value = %2d\n", Rs1, Registers[Rs1]);
		
	if(RegW) 	
		$display("The register R%0d updated new value = %2d\n", Rd, Registers[Rd]);
endtask

// -------------------- Main Control --------------------------
task MainControl(input [5:0] opcode);
	if(opcode==6'b000000 || opcode==6'b000001 || opcode==6'b000010)
		// R-type signals (AND, ADD, SUB)
		begin
			Reg2Src = 1'b1;
			Rs1W = 1'b0;
			RegW = 1'b1;
			ALUsrc = 1'b0;
			MemR = 1'b0;
			MemW = 1'b0;
			DataWB = 'b1;
			if(opcode==6'b000000) // AND
				ALUOp = `AND;
			else if(opcode==6'b000001) // ADD
				ALUOp = `ADD;
			else // SUB
				ALUOp = `SUB;
		end
	// I-type signals
	else if(opcode==6'b000011 || opcode==6'b000100 || opcode==6'b000101 || 
			opcode==6'b000111 || opcode==6'b001000 || opcode==6'b001001 || 
			opcode==6'b001010 || opcode==6'b001011)
			begin
				if(opcode==6'b000011) // ANDI
					begin
						Rs1W = 1'b0;
						RegW = 1'b1;
						ALUsrc = 1'b1;
						ExtOp = 1'b0;
						MemR = 1'b0;
						MemW = 1'b0;
						DataWB = 'b1;
						ALUOp = `AND;
					end
				else if(opcode==6'b000100) // ADDI
					begin
						Rs1W = 1'b0;
						RegW = 1'b1;
						ALUsrc = 1'b1;
						ExtOp = 1'b1;
						MemR = 1'b0;
						MemW = 1'b0;
						DataWB = 'b1;
						ALUOp = `ADD;
					end
				else if(opcode==6'b000101) // LW & LW.POI
					begin
						if(!IR[0])// if the least significant bit of mode = 0 then it is normal Load
							begin
								Rs1W = 1'b0;
								RegW = 1'b1;
								ALUsrc = 1'b1;
								ExtOp = 1'b1;
								AddSrc = 1'b0;
								MemR = 1'b1;
								MemW = 1'b0;
								DataWB = 'b0;
								ALUOp = `ADD;
							end
						else // there is a post increment on Rs1
							begin
						  		Rs1W = 1'b1;
								RegW = 1'b1;
								ALUsrc = 1'b1;
								ExtOp = 1'b1;
								AddSrc = 1'b0;
								MemR = 1'b1;
								MemW = 1'b0;
								DataWB = 'b0;
								ALUOp = `ADD;
							end	
					end
				else if(opcode==6'b000111) // SW
					begin
						Reg2Src = 1'b0;
						Rs1W = 1'b0;
						RegW = 1'b0;
						ALUsrc = 1'b1;
						ExtOp = 1'b1;
						DataSrc = 1'b1;
						AddSrc = 1'b0;
						MemR = 1'b0;
						MemW = 1'b1;
						ALUOp = `ADD;
					end
				else // Branch instructions
					begin
						Reg2Src = 1'b0;
						Rs1W = 1'b0;
						RegW = 1'b0;
						ALUsrc = 1'b0;
						ExtOp = 1'b1;
						MemR = 1'b0;
						MemW = 1'b0;
						ALUOp = `SUB;
					end	
			end
	// J-type signals
	else if(opcode==6'b001100 || opcode==6'b001101 || opcode==6'b001110)
		begin
			// common signals among all J-type instructions
			Rs1W = 1'b0;
			RegW = 1'b0;
			if(opcode==6'b001100) // JMP
				begin
					MemR = 1'b0;
					MemW = 1'b0;
				end
			else if(opcode==6'b001101) // CALL
				begin
					DataSrc = 1'b0;
					AddSrc = 1'b1;
					MemR = 1'b0;
					MemW = 1'b1;
				end 
			else // RET
				begin
					Reg2Src = 1'b0;
					AddSrc = 1'b1;
					MemR = 1'b1;
					MemW = 1'b0;
				end			
		end
	// S-type signals
	else if(opcode==6'b001111 || opcode==6'b010000)
		begin
			// common signals among all S-type instructions
			Rs1W = 1'b0;
			AddSrc = 1'b1;
			if(opcode==6'b001111) // PUSH
				begin
					Reg2Src = 1'b0;
					RegW = 1'b0;
					DataSrc = 1'b1;
					MemR = 1'b0;
					MemW = 1'b1;
				end
			else // POP
				begin
					RegW = 1'b1;
					MemR = 1'b1;
					MemW = 1'b0;
					DataWB = 'b0;
				end
		end
endtask	


// -------------------- ALU --------------------------
task ALU (input [1:0] ALUOp, input signed [31:0] op1, input signed [31:0] op2, output reg signed [31:0]result, output reg z, n, v);

	// compute result
	case(ALUOp)
		`AND: begin result = op1 & op2; end
		`ADD: begin result = op1 + op2; end
		`SUB: begin result = op1 - op2; end
	endcase
	
	// set flags
	z = result == 32'd0;
	n = result < 0;
	v = (op1[31] != op2[31]) && (result[31] != op1[31]);
/*	
	if (ALUOp == `ADD)
		v = (op1[31] == op2[31]) && (result[31] != op1[31]);
	else if (ALUOp == `SUB && op2[31] == 1'b1)
		v = 1'b1;
	else
		v = 1'b0;
*/
	$display("op1= %2d, op2=%2d\n", op1, op2);
	$display("ALU result= %2d, z=%b, n=%b, v=%b\n", result, z, n, v);
endtask	

///---------------------EXTENDER--------------- This task to extend the 16-bit immediate to be 32-bit as signed or unsigned extension
task Extender(input [15:0]imm, input ext_op,output signed [31:0]outExt); 
	if(ext_op)
		begin
			if(imm[15]==1'b1)
				outExt={16'hFFFF,imm}; // the sign extended is ones
			else
				outExt={16'b0,imm};
		end
	else
		outExt={16'b0,imm};
	
endtask
endmodule



module Test; // test bench for the final module 	 GeneralMod	module 	
 //Declarations of test inputs and outputs 
 
	reg clk;// every component in our data path synchronous by clock signal (sequantial circite )(when positive edge comes all component sensetive for clk signal)
	reg reset;
	
	wire zeroFlag, overflowFlag, negativeFlag;
	wire [31:0] ALUOut, dataOut, WBData;
	
	
	MultiCycleProcessor MCP(clk, reset, zeroFlag, overflowFlag, negativeFlag, ALUOut, dataOut, WBData);	  

	//Give clk values and change it every 0.5 units of time
	//and observe the output to tacks binary value for instructions to complete testing


	initial clk = 0;	
	always #10 clk = ~clk;
	initial #2500 $finish; 			
endmodule