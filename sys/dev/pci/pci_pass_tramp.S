/*-
 * Copyright (c) 2014, Matthew Macy <kmacy@FreeBSD.ORG>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *	$FreeBSD$
 *
 */
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
