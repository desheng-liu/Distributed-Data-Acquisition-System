// FiFo.c
// Runs on LM4F120/TM4C123
// Provide functions that implement the Software FiFo Buffer
// Last Modified: 11/11/2021 
// Student names: change this to your names or look very silly
#include <stdint.h>

// Declare state variables for FiFo
//        size, buffer, put and get indexes

int32_t static Fifo[9];
int static PutIndex;
int static GetIndex; 
// *********** FiFo_Init**********
// Initializes a software FIFO of a
// fixed size and sets up indexes for
// put and get operations
void Fifo_Init() {
//Complete this
	PutIndex = 0;
	GetIndex = 0;
}

// *********** FiFo_Put**********
// Adds an element to the FIFO
// Input: Character to be inserted
// Output: 1 for success and 0 for failure
//         failure is when the buffer is full
uint32_t Fifo_Put(char data){ 
  //Complete this routine
		if((PutIndex+1)%10 == GetIndex){
			return 0;
		}		
			
		Fifo[PutIndex] = data; //from other TM4c ?? 
		PutIndex = (PutIndex+1)%10;
		return(1);
}

// *********** Fifo_Get**********
// Gets an element from the FIFO
// Input: Pointer to a character that will get the character read from the buffer
// Output: 1 for success and 0 for failure
//         failure is when the buffer is empty
uint32_t Fifo_Get(char *datapt){
  //Complete this routine
	 if(PutIndex == GetIndex)
	 {
		 return 0;
	 }
	  *datapt = Fifo[GetIndex];
		GetIndex = (GetIndex+1)%10;
		return(1);
}




