#include <sys/syscall.h>

#include <machine/asmacros.h>

#include "assym.s"
/*
 * Initial frame:
 *
 * trap handler
 * mask addr
 * %rax
 * %rbx
 * %rsp
 * %rip
 */	
        .text
NON_GPROF_ENTRY(trapcode)
	popq	%rax			;		/* address of trap handler */
	PUSH_FRAME				;
	movq	%rsp,%rdi		;		/* pass trapframe */
    call    *(%rax)			;       /* call trap handler */
	POP_FRAME				;

	movb	$0,(%rsp)		; 		/* enable interrupts 	*/
	addq	8,%rsp			;		/* skip over mask addr  */
	movq	16(%rsp),%rax	;		/* old stack pointer    */
	movq	24(%rsp),%rbx	;		/* old ip				*/
	movq	%rbx,8(%rax)	;		/* store old ip	on old sp */
	popq	%rax			;		/* restore original rax */
	popq	%rbx			;		/* restore original rbx */	
	popq	%rsp			;   	/* restore original stack */
	addq	-8,%rsp			;		/* make top of stack old ip */
	retq					;		/* return to original ctx */
    ALIGN_TEXT
etrapcode:

    .data
    .globl  sztrapcode
sztrapcode:
    .long   etrapcode-trapcode
