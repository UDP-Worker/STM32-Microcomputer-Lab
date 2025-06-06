    AREA MYCODE, CODE, READONLY
    ENTRY

__main
    EXPORT __main

ADD_START
    LDR R0, =data1        
    LDR R1, =data2        
    LDR R2, =0x20001000    
    MOV R3, #0             
    MOV R4, #16 

ADD_LOOP
    LDRB R5, [R0], #1
    LDRB R6, [R1], #1
    ADDS R7, R5, R6       
    ADDS R7, R7, R3 
    MOVS R3, R7, LSR #8 ;
	AND R7, R7, #0xFF
    STRB R7, [R2], #1
    SUBS R4, R4, #1
    BNE ADD_LOOP

SUB_START
    LDR R0, =data1         
    LDR R1, =data2        
    LDR R2, =0x20002000    
    MOV R3, #0             
    MOV R4, #16          

SUB_LOOP
    LDRB R5, [R0], #1
    LDRB R6, [R1], #1
    SUBS R7, R5, R6        
    SUBS R7, R7, R3  
    MOVS R3, R7, LSR #8 ; 
	AND R7, R7, #0xFF	
    STRB R7, [R2], #1
    SUBS R4, R4, #1
    BNE SUB_LOOP

STOP
	B STOP

    ALIGN 4
data1   DCB 254,25,43,65,19,234,34,123,198,45,87,90,102,9,56,100
data2   DCB 250,20,12,216,19,60,29,178,190,1,20,38,128,6,200,39

    END
