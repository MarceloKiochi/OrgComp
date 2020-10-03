#include <stdio.h>
#include <stdlib.h>

#define RegDst ((SinaisUC) & 0x3)
#define RegWrite ((SinaisUC >> 2) & 0x1)
#define ALUSrcA ((SinaisUC >> 3) & 0x1)
#define ALUSrcB ((SinaisUC >> 4) & 0x3)
#define ALUOp ((SinaisUC >> 6) & 0x3)
#define PCSource ((SinaisUC >> 8) & 0x3)
#define PCWriteCond ((SinaisUC >> 10) & 0x1)
#define PCWrite ((SinaisUC >> 11) & 0x1)
#define IorD ((SinaisUC >> 12) & 0x1)
#define MemRead ((SinaisUC >> 13) & 0x1)
#define MemWrite ((SinaisUC >> 14) & 0x1)
#define BNE ((SinaisUC >> 15) & 0x1)
#define IRWrite ((SinaisUC >> 16) & 0x1)
#define MemtoReg ((SinaisUC >> 17) & 0x3)

#define IR_FNC_COD ((RegAux[IR].out) & 0x3f)
#define IR_IMMED ((RegAux[IR].out) & 0xffff)
#define IR_JMP ((RegAux[IR].out) & 0x3ffffff)
#define IR_RD ((RegAux[IR].out >> 11) & 0x1f)
#define IR_RT ((RegAux[IR].out >> 16) & 0x1f)
#define IR_RS ((RegAux[IR].out >> 21) & 0x1f)
#define IR_OP ((RegAux[IR].out >> 26) & 0x3f)

#define FNC_COD_ADD 32 
#define FNC_COD_SUB 34
#define FNC_COD_SLT 42
#define FNC_COD_AND 36
#define FNC_COD_OR 37

#define ADD 0
#define SUB 1
#define SLT 2
#define AND 3
#define OR 4

#define S0 (s & 1)
#define S1 ((s & 2) >> 1)
#define S2 ((s & 4) >> 2)
#define S3 ((s & 8) >> 3)

#define OP0 (opCode & 1)
#define OP1 ((opCode & 2) >> 1)
#define OP2 ((opCode & 4) >> 2)
#define OP3 ((opCode & 8) >> 3)
#define OP4 ((opCode & 16) >> 4)
#define OP5 ((opCode & 32) >> 5)

#define RAMTAM 128

typedef struct r Register;

enum {
	PC,
	IR,
	MDR,
	A,
	B,
	ALUOut,
	QntReg
};

enum{
	SUCCESS,
	UC_ERROR,
	MEM_ERROR,
	BCO_REG_ERROR,
	ALU_CTRL_ERROR
};

struct r{
	int in, out;
};

int SinaisUC;

int lastInstructionPosition;

Register RegAux[QntReg];

Register BCO_REG[32];

unsigned char RAM[RAMTAM];


int MUX_IorD_out;
int MUX_MemtoReg_out;
int MUX_ALUSrcB_out;

int SHT_BEQ_out;
int Sign_Extend_out;

int MUX_RegDst_out;
int MUX_ALUSrcA_out;
int MUX_PCSource_out;
int SHT_JMP_out;

int ALU_Zero;
int ALU_out;
int ALU_CTRL_out;

int MUX_BNE_out;

int PCWriteBit;

int UC(){
    static char s = 0;
    static char n = 0;
    char opCode  = (IR_OP);
    s = n;
    //printf("%d\n", s);
    //printf("opCode : %d\n",opCode);
    //printf("opCode : %d\n",RegAux[IR].out);
    n = 0;
    n = (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & !OP4 & !OP5) | //bne
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & !OP3 & !OP4 & !OP5) | //beq
        (S0 & !S1 & !S2 & !S3 & !OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //j
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //jal
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jr
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jalr
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & OP3 & !OP4 & !OP5) | // andi
        (!S0 & S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & OP3 & !OP4 & !OP5) | // Addi
        (!S0 & !S1 & S2 & S3);
    n = n << 1;
    n = (n) |
        (S0 & S1 & !S2 & !S3) |
        ((!S0) & S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & OP3 & !OP4 & OP5) | // SW
        ((!S0) & S1 & S2 & !S3) |
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //jal
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jr
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jalr
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & OP3 & !OP4 & !OP5) | // andi
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & !OP3 & !OP4 & !OP5); // TIPO-R
    n = n << 1;
    n = (n) |
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jr
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jalr
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & !OP4 & !OP5) | //bne
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & !OP3 & !OP4 & !OP5) |// TIPO-R
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & OP5) | // LW
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & OP3 & !OP4 & OP5) | // SW
        (S0 & !S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & OP3 & !OP4 & !OP5) | // andi
        ((!S0) & S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & OP5) | // lw
        ((!S0) & S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & OP3 & !OP4 & !OP5) | // ADDI
        (!S0 & !S1 & S2 & S3) |
        ((!S0) & S1 & S2 & !S3);
    n = n << 1;
    n = (n) |
        (S0 & !S1 & !S2 & !S3 & !OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //j
        (S0 & !S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & !OP5) | //jal
        (S0 & !S1 & !S2 & !S3 & OP0 & !OP1 & OP2 & !OP3 & OP4 & !OP5) | // jalr
        ((!S0) & S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & !OP3 & !OP4 & OP5) | // lw
        ((!S0) & S1 & !S2 & !S3 & OP0 & OP1 & !OP2 & OP3 & !OP4 & OP5) | // Sw
        ((!S0) & S1 & !S2 & !S3 & !OP0 & !OP1 & !OP2 & OP3 & !OP4 & !OP5) | // ADDI
        ((!S0) & S1 & S2 & !S3) |
        ((!S0) & !S1 & S2 & S3) |
        ((!S0) & !S1 & !S2 & !S3);

    SinaisUC = 0;
    SinaisUC = (S3&S2&!S1&S0) | (S3&S2&S1&S0);
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (!S3&S2&!S1&!S0);
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (!S3&!S2&!S1&!S0);
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (S3&!S2&S1&!S0);
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (!S3&S2&!S1&S0);
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&!S1&!S0) | (!S3&!S2&S1&S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&S1&S0) | (!S3&S2&!S1&S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&!S1&!S0) | (S3&!S2&!S1&S0) | (S3&S2&!S1&S0) | (S3&S2&S1&!S0) | (S3&S2&S1&S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&!S2&!S1&!S0) | (S3&!S2&S1&!S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&!S2&!S1&S0) | (S3&S2&!S1&S0) | (S3&S2&S1&!S0) | (S3&S2&S1&S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&!S2&!S1&!S0) | (S3&!S2&S1&!S0) | (S3&S2&S1&S0) | (S3&S2&S1&!S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&S2&S1&!S0) | (S3&S2&!S1&!S0)); //aluop1
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&!S2&!S1&!S0) | (S3&!S2&S1&!S0) | (S3&S2&!S1&!S0)); //aluop0
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&!S1&S0) | (S3&S2&!S1&!S0) | (!S3&!S2&S1&!S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&!S1&!S0) | (!S3&!S2&!S1&S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&!S2&S1&!S0) | (!S3&S2&S1&!S0) | (S3&!S2&!S1&!S0) | (S3&!S2&S1&!S0) | (S3&S2&!S1&!S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((!S3&S2&!S1&!S0) | (!S3&S2&S1&S0) | (S3&!S2&S1&S0) | (S3&S2&!S1&S0) | (S3&S2&S1&S0));
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | ((S3&S2&!S1&S0) ); //regdst1
    SinaisUC = SinaisUC << 1;
    SinaisUC = SinaisUC | (!S3&S2&S1&S0); //regdst0
    //printf("%d\n",s);

      return !((!OP5 & !OP4 & !OP3 & !OP2 & !OP1 & !OP0) |
        (!OP5 & !OP4 & !OP3 & OP2 & !OP1 & !OP0) |
        (!OP5 & !OP4 & !OP3 & OP2 & !OP1 & OP0) |
        (!OP5 & !OP4 & !OP3 & !OP2 & OP1 & !OP0) |
        (!OP5 & !OP4 & !OP3 & !OP2 & OP1 & OP0) |
        (!OP5 & OP4 & !OP3 & OP2 & !OP1 & !OP0) |
        (!OP5 & OP4 & !OP3 & OP2 & !OP1 & OP0) |
        (OP5 & !OP4 & !OP3 & !OP2 & OP1 & OP0) |
        (OP5 & !OP4 & OP3 & !OP2 & OP1 & OP0) |
        (!OP5 & !OP4 & OP3 & !OP2 & !OP1 & !OP0) |
        (!OP5 & !OP4 & OP3 & OP2 & !OP1 & !OP0));
}

void MUX_IorD(){
	if(IorD == 0x0){
		MUX_IorD_out = RegAux[PC].out;
	} else {
		MUX_IorD_out = RegAux[ALUOut].out;
	}
}

void MUX_RegDst(){
	//printf("RG_DST = %d\n", RegDst);
	if(RegDst==0){
		MUX_RegDst_out = IR_RT;
	}
	else if(RegDst==1){
		MUX_RegDst_out = IR_RD;
	}
	else{
		MUX_RegDst_out = 31;
	}
	// printf("MUX_RegDst_out = %d\n", MUX_RegDst_out);
}

void MUX_MemtoReg(){
	if(MemtoReg == 0x0){
		//printf("AQUI\n");
		MUX_MemtoReg_out = RegAux[ALUOut].out;
		//printf("aluout = %d\n", RegAux[ALUOut].out);
	} else if(MemtoReg == 0x1){
		MUX_MemtoReg_out = RegAux[MDR].out;
	} else {
		MUX_MemtoReg_out = RegAux[PC].out;
	}
	// printf("MUX_MemtoReg_out = %d\n", MUX_MemtoReg_out);
}

void MUX_ALUSrcA(){
	if(ALUSrcA==0){
		MUX_ALUSrcA_out = RegAux[PC].out;
	}
	else{
		MUX_ALUSrcA_out = RegAux[A].out;
	}
	//MUX_ALUSrcA_out = RegAux[PC].out;
}

void MUX_ALUSrcB(){
	//printf("alusrcB = %d\n", ALUSrcB);
	if(ALUSrcB == 0){
		MUX_ALUSrcB_out = RegAux[B].out;
	} else if(ALUSrcB == 1){
		MUX_ALUSrcB_out = 4;
	} else if(ALUSrcB == 2){
		MUX_ALUSrcB_out = Sign_Extend_out;
	} else {
		//printf("AQUI\n");
		MUX_ALUSrcB_out = SHT_BEQ_out;
	}
	//MUX_ALUSrcB_out = 4;

	//printf("b out = %d\n", MUX_ALUSrcB_out);

}

void MUX_PCSource(){
	//printf("aluout = %d\n", ALU_out);
	if(PCSource==0){
		RegAux[PC].in = ALU_out;
	}
	else if(PCSource==1){
		RegAux[PC].in = RegAux[ALUOut].out;
	}
	else if(PCSource==2){
		RegAux[PC].in = SHT_JMP_out;
	}
	else{
		RegAux[PC].in = RegAux[A].out;
	}
	//RegAux[PC].in = ALU_out;
}

void MUX_BNE(){
	if(BNE == 0){
		MUX_BNE_out = ALU_Zero;
	} else{
		MUX_BNE_out = !ALU_Zero;
	}
}

void Sign_Extend(){
	int sign = IR_IMMED >> 15;
	if (sign == 1){
		sign = ((-1) << 15);
	}
	Sign_Extend_out = IR_IMMED | sign;
}

void SHT_BEQ(){
	SHT_BEQ_out = Sign_Extend_out << 2;
}

void SHT_JMP(){
	SHT_JMP_out = IR_JMP << 2;
}

int ALU_CTRL(){
	// printf("aluop = %d\n", ALUOp);
	// printf("ir fnc cod = %d\n", IR_FNC_COD);


	if(ALUOp == 0){
		ALU_CTRL_out = ADD;
	}
	else if(ALUOp == 1){
		ALU_CTRL_out = SUB;
	}
	else if(ALUOp == 2){


		if(IR_FNC_COD == FNC_COD_ADD){
			ALU_CTRL_out = ADD;
		}
		else if(IR_FNC_COD == FNC_COD_SUB){
			ALU_CTRL_out = SUB;
		}
		else if(IR_FNC_COD == FNC_COD_SLT){
			ALU_CTRL_out = SLT;
		}
		else if(IR_FNC_COD == FNC_COD_AND){
			//printf("algo\n");
			ALU_CTRL_out = AND;
		}
		else if(IR_FNC_COD == FNC_COD_OR){
			ALU_CTRL_out = OR;
		}
		else{
			//printf("entro no else = %d\n", IR_FNC_COD);
			return ALU_CTRL_ERROR;
		}
	}
	else if(ALUOp == 3){
		ALU_CTRL_out = AND;
	}
	else{
		return ALU_CTRL_ERROR;
	}

	return SUCCESS;

	//ALU_CTRL_out = ADD;
}

void ALU(){

	if(ALU_CTRL_out == ADD){
		ALU_out = MUX_ALUSrcA_out + MUX_ALUSrcB_out;		
	}
	else if(ALU_CTRL_out == SUB){
		ALU_out = MUX_ALUSrcA_out - MUX_ALUSrcB_out;
	}
	else if(ALU_CTRL_out == SLT){
		ALU_out = MUX_ALUSrcA_out < MUX_ALUSrcB_out;
	}
	else if(ALU_CTRL_out == AND){
		ALU_out = MUX_ALUSrcA_out & MUX_ALUSrcB_out;
		//printf("ALU_out = %d, ALUA = %d, ALUB = %d\n", ALU_out, MUX_ALUSrcA_out, MUX_ALUSrcB_out);
	}
	else if(ALU_CTRL_out == OR){
		ALU_out = MUX_ALUSrcA_out | MUX_ALUSrcB_out;
	}

	if(ALU_out == 0){
		ALU_Zero = 1;
	}
	else{
		ALU_Zero = 0;
	}

	RegAux[ALUOut].in = ALU_out;
	// printf("aluctrl = %d\n", ALU_CTRL_out);
	// printf("alu: %d, %d %d\n", ALU_out, MUX_ALUSrcA_out, MUX_ALUSrcB_out);
}

int Banco_Reg(){

	if(IR_RS < 0 || IR_RS > 31 || IR_RT < 0 || IR_RT > 31){
		return BCO_REG_ERROR;
	}

	RegAux[A].in = BCO_REG[IR_RS].out;

	RegAux[B].in = BCO_REG[IR_RT].out;

	if(RegWrite == 1){
		//printf("ENTROU - %d\n", MUX_RegDst_out);
		if(MUX_RegDst_out <= 0 || MUX_RegDst_out > 31){
			return BCO_REG_ERROR;
		}
		//printf("memtoreg out = %d\n", MUX_MemtoReg_out);
		//printf("regdst = %d\n", MUX_RegDst_out);
		BCO_REG[MUX_RegDst_out].in = MUX_MemtoReg_out;
	}

	return SUCCESS;
}

int Mem(){
	if(MUX_IorD_out < 0 || MUX_IorD_out >= RAMTAM || MUX_IorD_out%4 != 0){
		return MEM_ERROR;
	}
	int *aux = (int *) &RAM[MUX_IorD_out];
	if(MemRead){
		RegAux[MDR].in = *aux;
		RegAux[IR].in = *aux;		
	}

	if(MemWrite){
		if(MUX_IorD_out <= lastInstructionPosition)
			return MEM_ERROR;
		*aux = RegAux[B].out;
	}

	return SUCCESS;
}

void PCWriteFNC(){
	PCWriteBit = (MUX_BNE_out & PCWriteCond) | PCWrite;
}

void ClockUp(){
	//PCWriteBit = 1;
	//printf("pcwritebit = %d\n", PCWriteBit);
	//printf("pc in = %d\n", RegAux[PC].in);
	if(PCWriteBit && RegAux[IR].in != -1){
		RegAux[PC].out = RegAux[PC].in;
	}

	if(IRWrite){
		RegAux[IR].out = RegAux[IR].in;

	}

	if(RegWrite){
		BCO_REG[MUX_RegDst_out].out = BCO_REG[MUX_RegDst_out].in;
	}

	RegAux[MDR].out = RegAux[MDR].in;
	RegAux[A].out = RegAux[A].in;
	RegAux[B].out = RegAux[B].in;
	RegAux[ALUOut].out = RegAux[ALUOut].in;
}

int main(){
	int i = 1; //, j=0;
	unsigned int inst;
	int erro = 0;
	int memPosition = 0;
	unsigned int *aux;

	FILE *infile = fopen("code.bin", "r");

	for(i=0; i<RAMTAM; i++){
		RAM[i] = 0;
	}

	for(i=0; i<32; i++){
		BCO_REG[i].in = 0;
		BCO_REG[i].out = 0;
	}

	for(i=0; i<QntReg; i++){
		RegAux[i].in = 0;
		RegAux[i].out = 0;
	}

	while(fscanf(infile, "%d", &inst) != EOF){
		//printf("%d\n", inst);
		aux = (int *)&RAM[memPosition];
		*aux = inst; 
		memPosition = memPosition + 4;
	}

	lastInstructionPosition = memPosition - 4;
	// aux = (int *)&RAM[memPosition];
	// *aux = -1;

	// for(i=0; i<128; i=i+4){
	// 	printf("RAM[%d] = %u\n", i, *((int *)&RAM[i]));
	// 	fflush(stdout);
	// }

	BCO_REG[29].out = RAMTAM;	// Stack Pointer = ultima posicao de memoria + 1

	while(1){
		//printf("%d\n", RegAux[PC].out);
		// for(i=0; i<4; i++){
		// 	printf("BCO_REG[%d] = %d\n", i, BCO_REG[i].out);
		// 	fflush(stdout);
		// }
		// for(i=0; i<16; i=i+4){
		// 	printf("RAM[%d] = %d\n", i, *((int *)&RAM[i]));
		// 	fflush(stdout);
		// }
		// printf("SP = %d\n", BCO_REG[29].out);
		// scanf("%d", &SinaisUC);
		// // printf("IR IMMED = %d\n", IR_IMMED);
		// printf("RA = %d\n", BCO_REG[31].out);
		// //printf("c = %d\n", j);
		  printf("pc = %d\n", RegAux[PC].out);
		 // printf("S0 = %d 	S1 = %d 	S2 = %d\n", BCO_REG[16].out, BCO_REG[17].out, BCO_REG[18].out);
		 // printf("SP = %d, Memoria = %d\n", BCO_REG[29].out, RAM[BCO_REG[29].out]);
		 //scanf("%d", &algo);
		// printf("IR = %d\n", RegAux[IR].out);
		// printf("%d\n", RegDst);
		// printf("%d\n", RegWrite);
		// printf("%d\n", MemtoReg);
		//j++;
		// printf("rt = %d, rs = %d, rd = %d\n", IR_RT, IR_RS, IR_RD);
		erro = UC();
		if(erro != 0){ break; }
		MUX_IorD();
		erro = Mem();
		if(erro != 0){ break; }
		MUX_RegDst();
		MUX_MemtoReg();
		erro = Banco_Reg();
		if(erro != 0){ break; }
		Sign_Extend();
		SHT_BEQ();
		SHT_JMP();
		MUX_ALUSrcA();
		MUX_ALUSrcB();
		erro = ALU_CTRL();
		if(erro != 0){ break; }
		ALU();
		MUX_PCSource();
		MUX_BNE();
		PCWriteFNC();
		ClockUp();
	}

	//printf("ola");
	//printf("ERRO = %d\n", erro);

	printf("\nStatus da Saída: ");

	switch(erro){
		case UC_ERROR:
			printf("Termino devido a tentativa de execucao de instrucao invalida. Codigo de operacao = \n");
			for(i=0; i<6; i++){
				printf("%d", ((SinaisUC) >> (5-i))&0x1);
			} 
			printf(" (%d)", SinaisUC >> 25);
			break;
		case MEM_ERROR:
			printf("Termino devido a acesso invalido de memoria. Posicao acessada = %d\n", MUX_IorD_out);
			break;
		case BCO_REG_ERROR:
			printf("Termino devido a acesso invalido ao banco de registradores. Numero de registrador acessado = %d\n", MUX_RegDst_out);
			break;
		case ALU_CTRL_ERROR:
			printf("Termino devido a operacao invalida da ULA. Codigo de Funcao = \n");
			for(i=0; i<6; i++){
				printf("%d", ((IR_FNC_COD) >> (5-i))&0x1);
			} 
			printf(" (%d)", IR_FNC_COD);
			break;
	}

	printf("\n");

	printf("PC = %d 	", RegAux[PC].out);
	printf("IR = %u 	", RegAux[IR].out);
	printf("MDR = %d\n", RegAux[MDR].out);
	printf("A = %d 	", RegAux[A].out);
	printf("B = %d 	", RegAux[B].out);
	printf("ALUOut = %d\n", RegAux[ALUOut].out);

	printf("Controle = ");
	for(i=0; i<32; i++){
		printf("%d", ((SinaisUC) >> (31-i))&0x1);
	}
	printf(" (%d)", SinaisUC);

	printf("\n\n");

	for(i=0; i<32; i=i+4){
		printf("RAM[%d] = %u 	RAM[%d] = %u 	RAM[%d] = %u 	RAM[%d] = %u",
			i, *((int *)&RAM[i]), i+32, *((int *)&RAM[i+32]), i+64, *((int *)&RAM[i+64]), i+96, *((int *)&RAM[i+96]));
		printf("\n");
	}

	printf("\n");

	// for(i=0; i<8; i++){
	// 	printf("BCO_REG[%d] = %d", i, BCO_REG[i].out);
	// 	fflush(stdout);
	// }

	printf("BCO_REG[%d](zero) = %d 	BCO_REG[%d](t0) = %d 	BCO_REG[%d](s0) = %d 	BCO_REG[%d](t8) = %d", 	
		0, BCO_REG[0].out, 8, BCO_REG[8].out, 16, BCO_REG[16].out, 24, BCO_REG[24].out);
	printf("\n");
	printf("BCO_REG[%d](at) = %d 	BCO_REG[%d](t1) = %d 	BCO_REG[%d](s1) = %d 	BCO_REG[%d](t9) = %d", 	
		1, BCO_REG[1].out, 9, BCO_REG[9].out, 17, BCO_REG[17].out, 25, BCO_REG[25].out);
	printf("\n");
	printf("BCO_REG[%d](v0) = %d 	BCO_REG[%d](t2) = %d 	BCO_REG[%d](s2) = %d 	BCO_REG[%d](k0) = %d", 	
		2, BCO_REG[2].out, 10, BCO_REG[10].out, 18, BCO_REG[18].out, 26, BCO_REG[26].out);
	printf("\n");
	printf("BCO_REG[%d](v1) = %d 	BCO_REG[%d](t3) = %d 	BCO_REG[%d](s3) = %d 	BCO_REG[%d](k1) = %d", 	
		3, BCO_REG[3].out, 11, BCO_REG[11].out, 19, BCO_REG[19].out, 27, BCO_REG[27].out);
	printf("\n");
	printf("BCO_REG[%d](a0) = %d 	BCO_REG[%d](t4) = %d 	BCO_REG[%d](s4) = %d 	BCO_REG[%d](gp) = %d", 	
		4, BCO_REG[4].out, 12, BCO_REG[12].out, 20, BCO_REG[20].out, 28, BCO_REG[28].out);
	printf("\n");
	printf("BCO_REG[%d](a1) = %d 	BCO_REG[%d](t5) = %d 	BCO_REG[%d](s5) = %d 	BCO_REG[%d](sp) = %d", 	
		5, BCO_REG[5].out, 13, BCO_REG[13].out, 21, BCO_REG[21].out, 29, BCO_REG[29].out);
	printf("\n");
	printf("BCO_REG[%d](a2) = %d 	BCO_REG[%d](t6) = %d 	BCO_REG[%d](s6) = %d 	BCO_REG[%d](fp) = %d", 	
		6, BCO_REG[6].out, 14, BCO_REG[14].out, 22, BCO_REG[22].out, 30, BCO_REG[30].out);
	printf("\n");
	printf("BCO_REG[%d](a3) = %d 	BCO_REG[%d](t7) = %d 	BCO_REG[%d](s7) = %d 	BCO_REG[%d](ra) = %d", 	
		7, BCO_REG[7].out, 15, BCO_REG[15].out, 23, BCO_REG[23].out, 31, BCO_REG[31].out);
	printf("\n\n");

	return 0;
}
