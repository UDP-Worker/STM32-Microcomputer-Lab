  AREA MYCODE, CODE, READONLY
  ENTRY
      
__main
  EXPORT __main
    
    LDR   r0, =data          
    LDR   r1, =0x20001000    
    MOV   r2, #16            
copy_forward
    LDRB  r3, [r0], #1       
    STRB  r3, [r1], #1       
    SUBS  r2, r2, #1         
    BNE   copy_forward       

    
    LDR   r0, =data          
    ADD   r0, r0, #16        
    LDR   r1, =0x20002000    
    MOV   r2, #16            
copy_reverse
    LDRB  r3, [r0, #-1]!     
    STRB  r3, [r1], #1       
    SUBS  r2, r2, #1
    BNE   copy_reverse

end_loop
    B     end_loop           

  ; finish the code
  
  ALIGN 4
data DCB 254,25,43,65,19,234,34,123,198,45,87,90,102,9,56,100
  
  END
