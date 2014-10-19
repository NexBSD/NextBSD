.include <bsd.init.mk>

LIB=	ukern

#	
#	kern_event.o		\



PN_KERN_COMMON_OBJS +=	\
	kern_acct.o			\
	kern_clock.o		\
	kern_clocksource.o	\
	kern_condvar.o		\
	kern_cons.o			\
	kern_cpu.o			\
	kern_cpuset.o		\
	kern_conf.o			\
	kern_context.o		\
	kern_descrip.o		\
	kern_environment.o	\
	kern_et.o			\
	kern_exit.o			\
	kern_fail.o			\
	kern_fork.o			\
	kern_hhook.o		\
	kern_idle.o			\
	kern_intr.o			\
	kern_jail.o			\
	kern_khelp.o		\
	kern_kthread.o		\
	kern_linker.o		\
	kern_lock.o			\
	kern_lockf.o		\
	kern_loginclass.o	\
	kern_malloc.o		\
	kern_mib.o			\
	kern_mbuf.o			\
	kern_module.o		\
	kern_mtxpool.o		\
	kern_mutex.o		\
	kern_ntptime.o		\
	kern_osd.o			\
	kern_physio.o		\
	kern_proc.o			\
	kern_prot.o			\
	kern_rangelock.o	\
	kern_resource.o		\
	kern_rmlock.o		\
	kern_rwlock.o		\
	kern_shutdown.o		\
	kern_sig.o			\
	kern_switch.o		\
	kern_sx.o			\
	kern_synch.o		\
	kern_sysctl.o		\
	kern_tc.o			\
	kern_thread.o		\
	kern_time.o			\
	kern_timeout.o		\
	kern_umtx.o			\
	kern_uuid.o			\
	link_elf.o			\
	link_elf_obj.o		\
	md5c.o				\
	p1003_1b.o			\
	posix4_mib.c		\
	sched_ule.o			\
	subr_autoconf.o		\
	subr_blist.o		\
	subr_bus.o			\
	subr_bus_dma.o		\
	subr_bufring.o		\
	subr_capability.o	\
	subr_clock.o		\
	subr_counter.o		\
	subr_devstat.o		\
	subr_disk.o			\
	subr_eventhandler.o	\
	subr_hash.o			\
	subr_hints.o		\
	subr_kdb.o			\
	subr_kobj.o			\
	subr_lock.o			\
	subr_log.o			\
	subr_module.o		\
	subr_msgbuf.o		\
	subr_param.o		\
	subr_pcpu.o			\
	subr_pctrie.o		\
	subr_power.o		\
	subr_prf.o			\
	subr_rtc.o			\
	subr_rman.o			\
	subr_sbuf.o			\
	subr_sglist.o		\
	subr_smp.o			\
	subr_sleepqueue.o	\
	subr_stack.o		\
	subr_taskqueue.o	\
	subr_turnstile.o	\
	subr_unit.o			\
	subr_uio.o			\
	subr_vmem.o			\
	sys_capability.o	\
	sys_generic.o		\
	sys_procdesc.o		\
	sys_process.o		\
	sys_socket.o		\
	tty.o				\
	tty_info.c			\
	tty_inq.c			\
	tty_outq.c			\
	tty_pts.c			\
	tty_tty.c			\
	tty_ttydisc.c		\
	tty_ttydisc.o		\
	uipc_accf.o			\
	uipc_debug.o		\
	uipc_mbuf.o			\
	uipc_mbuf2.o		\
	uipc_domain.o		\
	uipc_sockbuf.o		\
	uipc_socket.o		\
	uipc_usrreq.o		\
	vfs_bio.o			\
	vfs_cache.o			\
	vfs_cluster.o		\
	vfs_default.o		\
	vfs_export.o		\
	vfs_init.o			\
	vfs_lookup.o		\
	vfs_mount.o			\
	vfs_mountroot.o		\
	vfs_subr.o			\
	vfs_syscalls.o		\
	vfs_vnops.o	


PN_VM_COMMON_OBJS +=	\
	default_pager.o		\
	device_pager.o		\
	phys_pager.o		\
	sg_pager.o			\
	swap_pager.o		\
	uma_dbg.o			\
	uma_core.o			\
	vm_fault.o			\
	vm_glue.o			\
	vm_init.o			\
	vm_kern.o			\
	vm_meter.o			\
	vm_map.o			\
	vm_mmap.o			\
	vm_object.o			\
	vm_page.o			\
	vm_pageout.o		\
	vm_pager.o			\
	vm_phys.o			\
	vm_radix.o			\
	vm_reserv.o			\
	vm_unix.o			\
	vm_zeroidle.o		\
	vnode_pager.o	

PN_FS_COMMON_OBJS +=		\
	dead_vnops.o	\
	devfs_devs.o	\
	devfs_dir.o		\
	devfs_rule.o	\
	devfs_vfsops.o	\
	devfs_vnops.o	\
	geom_ctl.o		\
	geom_dev.o		\
	geom_disk.o		\
	geom_dump.o		\
	geom_event.o	\
	geom_io.o		\
	geom_kern.o		\
	geom_slice.o	\
	geom_subr.o		\
	geom_vfs.o


PN_NET_COMMON_OBJS +=		\
	bpf.o			\
	if.o			\
	if_clone.o		\
	if_dead.o		\
	if_llatbl.o		\
	if_media.o		\
	if_mib.o		\
	netisr.o		\
	pfil.o			\
	radix.o			\
	radix_mpath.o	\
	raw_cb.o		\
	raw_usrreq.o	\
	route.o			\
	rtsock.o		\
	if_loop.o		\
	if_ethersubr.o	


# v6 requires carp !!
PN_NETINET_COMMON_OBJS +=	\
	ip_carp.o		\
	if_ether.o		\
	igmp.o			\
	in.o			\
	in_mcast.o		\
	in_pcb.o		\
	in_proto.o		\
	in_rmx.o		\
	ip_encap.o		\
	ip_fastfwd.o		\
	ip_icmp.o		\
	ip_id.o			\
	ip_input.o		\
	ip_options.o		\
	ip_output.o		\
	raw_ip.o		\
	tcp_debug.o		\
	tcp_hostcache.o		\
	tcp_input.o		\
	tcp_lro.o		\
	tcp_offload.o		\
	tcp_output.o		\
	tcp_reass.o		\
	tcp_sack.o		\
	tcp_subr.o		\
	tcp_syncache.o		\
	tcp_timer.o		\
	tcp_timewait.o		\
	tcp_usrreq.o		\
	udp_usrreq.o		\
	cc.o			\
	cc_newreno.o

PN_NETINET6_COMMON_OBJS +=	\
	in6_proto.o		\
	dest6.o			\
	frag6.o			\
	icmp6.o			\
	in6.o			\
	in6_cksum.o		\
	in6_ifattach.o		\
	in6_mcast.o		\
	in6_pcb.o		\
	in6_pcbgroup.o		\
	in6_rmx.o		\
	in6_src.o		\
	ip6_forward.o		\
	ip6_id.o		\
	ip6_input.o		\
	ip6_mroute.o		\
	ip6_output.o		\
	mld6.o			\
	nd6.o			\
	nd6_nbr.o		\
	nd6_rtr.o		\
	raw_ip6.o		\
	route6.o		\
	scope6.o		\
	send.o			\
	udp6_usrreq.o


PN_LIBKERN_COMMON_OBJS =	\
	arc4random.o		\
	bcd.o			\
	strlen.o		\
	strlcpy.o

PN_DDB_COMMON_OBJS = 		\
	db_access.o				\
	db_break.o				\
	db_capture.o			\
	db_command.o			\
	db_examine.o			\
	db_expr.o				\
	db_input.o				\
	db_lex.o				\
	db_main.o				\
	db_output.o				\
	db_print.o				\
	db_ps.o					\
	db_run.o				\
	db_script.o				\
	db_sym.o				\
	db_thread.o				\
	db_textdump.o			\
	db_variables.o			\
	db_watch.o				\
	db_write_cmd.o
# v6 requires sha1
PN_OTHER_COMMON_OBJS =		\
	harvest.o		\
	sha1.o			\
	in_cksum.o		\
	bus_if.o		\
	clock_if.o		\
	device_if.o		\
	linker_if.o		\
	vnode_if.o		\
	timer.o			\
	vers.o			\
	watchdog.o		\
	led.o

# pn_init.o needs to be first
PN_SYS_COMMON_OBJS = 		\
	ukern_compat.o			\
	ukern_glue.o			\
	ukern_init.o			\
	ukern_init_main.o		\
	ukern_other.o			\
	ukern_private.o			\
	ukern_server_syscalls.o	\
	ukern_user_sysctl.o		\
	ukern_uipc_syscalls.o	

# 	ukern_veth.o		\


PN_MD_COMMON_OBJS = 		\
	autoconf.o				\
	busdma_bounce.o			\
	db_disasm.o				\
	db_interface.o			\
	db_trace.o				\
	identcpu.o				\
	io_apic.o				\
	intr_machdep.o			\
	stack_machdep.o			\
	tsc.o					\
	busdma_machdep.o		\
	fpu.o					\
	initcpu.o				\
	machdep.o				\
	mp_machdep.o			\
	pmap.o					\
	vm_machdep.o


PN_MD_ASM_OBJS = 		\
	support.o			\
	cpu_switch.o		

PN_MFILES = 		\
	bus_if.h 		\
	clock_if.h		\
	cpufreq_if.h	\
	device_if.h		\
	linker_if.h		\
	vnode_if.h

PN_PCI_MFILES = 	\
	acpi_if.h		\
	acpi_wmi_if.h	\
	isa_if.h		\
	pci_if.h		\
	pcib_if.h

PN_MFILES += ${PN_PCI_MFILES}

PN_ACPICA_OBJS +=	\
	OsdDebug.o		\
	OsdHardware.o	\
	OsdInterrupt.o	\
	OsdMemory.o		\
	OsdSchedule.o	\
	OsdStream.o		\
	OsdSynch.o		\
	OsdTable.o		\
	acpi.o			\
	acpi_acad.o		\
	acpi_battery.o	\
	acpi_cmbat.o	\
	acpi_cpu.o		\
	acpi_ec.o		\
	acpi_hpet.o		\
	acpi_lid.o		\
	acpi_package.o	\
	acpi_pci.o		\
	acpi_pci_link.o	\
	acpi_pcib.o		\
	acpi_pcib_acpi.o\
	acpi_pcib_pci.o	\
	acpi_perf.o		\
	acpi_powerres.o	\
	acpi_resource.o	\
	acpi_smbat.o	\
	acpi_thermal.o	\
	acpi_throttle.o	\
	acpi_timer.o	\
	dsargs.o		\
	dscontrol.o		\
	dsfield.o		\
	dsinit.o		\
	dsmethod.o		\
	dsmthdat.o		\
	dsobject.o		\
	dsopcode.o		\
	dsutils.o		\
	dswexec.o		\
	dswload.o		\
	dswload2.o		\
	dswscope.o		\
	dswstate.o		\
	evevent.o		\
	evglock.o		\
	evgpe.o			\
	evgpeblk.o		\
	evgpeinit.o		\
	evgpeutil.o		\
	evhandler.o		\
	evmisc.o		\
	evregion.o		\
	evrgnini.o		\
	evsci.o			\
	evxface.o		\
	evxfevnt.o		\
	evxfgpe.o		\
	evxfregn.o		\
	exconfig.o		\
	exconvrt.o		\
	excreate.o		\
	exdebug.o		\
	exdump.o		\
	exfield.o		\
	exfldio.o		\
	exmisc.o		\
	exmutex.o		\
	exnames.o		\
	exoparg1.o		\
	exoparg2.o		\
	exoparg3.o		\
	exoparg6.o		\
	exprep.o		\
	exregion.o		\
	exresnte.o		\
	exresolv.o		\
	exresop.o		\
	exstore.o		\
	exstoren.o		\
	exstorob.o		\
	exsystem.o		\
	exutils.o		\
	hwacpi.o		\
	hwesleep.o		\
	hwgpe.o			\
	hwpci.o			\
	hwregs.o		\
	hwregs.o		\
	hwsleep.o		\
	hwtimer.o		\
	hwvalid.o		\
	hwxface.o		\
	hwxfsleep.o		\
	nsaccess.o		\
	nsalloc.o		\
	nsarguments.o	\
	nsconvert.o		\
	nsdump.o		\
	nseval.o		\
	nsinit.o		\
	nsload.o		\
	nsnames.o		\
	nsobject.o		\
	nsparse.o		\
	nspredef.o		\
	nsprepkg.o		\
	nsrepair.o		\
	nsrepair2.o		\
	nssearch.o		\
	nsutils.o		\
	nswalk.o		\
	nsxfeval.o		\
	nsxfname.o		\
	nsxfobj.o		\
	psargs.o		\
	psloop.o		\
	psobject.o		\
	psopcode.o		\
	psopinfo.o		\
	psparse.o		\
	psscope.o		\
	pstree.o		\
	psutils.o		\
	pswalk.o		\
	psxface.o		\
	rsaddr.o		\
	rscalc.o		\
	rscreate.o		\
	rsdump.o		\
	rsdumpinfo.o	\
	rsinfo.o		\
	rsio.o			\
	rsirq.o			\
	rslist.o		\
	rsmemory.o		\
	rsmisc.o		\
	rsserial.o		\
	rsutils.o		\
	rsxface.o		\
	tbdata.o		\
	tbfadt.o		\
	tbfind.o		\
	tbinstal.o		\
	tbprint.o		\
	tbutils.o		\
	tbxface.o		\
	tbxfload.o		\
	tbxfroot.o		\
	utaddress.o		\
	utalloc.o		\
	utbuffer.o		\
	utcache.o		\
	utcopy.o		\
	utdebug.o		\
	utdecode.o		\
	utdelete.o		\
	uterror.o		\
	uteval.o		\
	utexcep.o		\
	utglobal.o		\
	uthex.o			\
	utids.o			\
	utinit.o		\
	utlock.o		\
	utmath.o		\
	utmisc.o		\
	utmutex.o		\
	utobject.o		\
	utosi.o			\
	utownerid.o		\
	utpredef.o		\
	utresrc.o		\
	utstate.o		\
	utstring.o		\
	utxface.o		\
	utxferror.o		\
	utxfinit.o

PN_X86_ACPICA_OBJS =	\
	acpi_if.o			\
	acpi_wmi_if.o		\
	OsdEnvironment.o	\
	acpi_apm.o			\
	acpi_machdep.o		\
	acpi_wakeup.o		\
	madt.o				\
	srat.o

PN_PCI_OBJS	= 		\
	pci_if.o		\
	pcib_if.o		\
	hostb_pci.o		\
	pci.o			\
	pci_pci.o		\
	pci_subr.o		\
	pci_user.o		\
	pcib_support.o

PN_X86_PCI_OBJS =	\
	isa_if.o		\
	legacy.o		\
	msi.o			\
	nexus.o			\
	pci_bus.o		\
	pci_cfgreg.o	\
	qpi.o

PN_EM_OBJS = 			\
	if_em.c				\
	if_lem.c			\
	e1000_80003es2lan.o	\
	e1000_82540.o		\
	e1000_82541.o		\
	e1000_82542.o		\
	e1000_82543.o		\
	e1000_82571.o		\
	e1000_82575.o		\
	e1000_ich8lan.o		\
	e1000_i210.o		\
	e1000_api.o			\
	e1000_mac.o			\
	e1000_manage.o		\
	e1000_nvm.o			\
	e1000_phy.o			\
	e1000_vf.o			\
	e1000_mbx.o			\
	e1000_osdep.o

PN_OPT_FILES = 			\
	opt_acpi.h		\
	opt_adaptive_lockmgrs.h		\
	opt_adaptive_mutexes.h		\
	opt_atalk.h		\
	opt_atpic.h		\
	opt_bootp.h		\
	opt_bpf.h		\
	opt_bus.h		\
	opt_config.h		\
	opt_clock.h		\
	opt_cpu.h		\
	opt_callout_profiling.h		\
	opt_cam.h		\
	opt_capsicum.h		\
	opt_carp.h		\
	opt_compat.h	\
	opt_config.h	\
	opt_core.h		\
	opt_debug_cluster.h	\
	opt_debug_lockf.h	\
	opt_device_polling.h	\
	opt_ddb.h		\
	opt_ffclock.h		\
	opt_gdb.h		\
	opt_geom.h		\
	opt_global.h		\
	opt_hwpmc_hooks.h	\
	opt_inet.h		\
	opt_inet6.h		\
	opt_init_path.h \
	opt_ipfw.h		\
	opt_ipsec.h		\
	opt_ipstealth.h		\
	opt_ipx.h		\
	opt_isa.h		\
	opt_kdb.h		\
	opt_kdtrace.h		\
	opt_kld.h		\
	opt_ktrace.h		\
	opt_kstack_max_pages.h		\
	opt_kstack_pages.h		\
	opt_kstack_usage_prof.h		\
	opt_kqueue.h		\
	opt_maxusers.h		\
	opt_mbuf_profiling.h	\
	opt_mbuf_stress_test.h	\
	opt_mpath.h		\
	opt_mprof.h		\
	opt_mrouting.h		\
	opt_msgbuf.h		\
	opt_netgraph.h		\
	opt_no_adaptive_rwlocks.h		\
	opt_no_adaptive_sx.h		\
	opt_ntp.h		\
	opt_ofed.h		\
	opt_param.h		\
	opt_panic.h		\
	opt_pcbgroup.h	\
	opt_posix.h		\
	opt_pf.h		\
	opt_printf.h	\
	opt_procdesc.h	\
	opt_random.h	\
	opt_rootdevname.h	\
	opt_route.h		\
	opt_rss.h		\
	opt_sched.h		\
	opt_sctp.h		\
	opt_sleepqueue_profiling.h		\
	opt_smp.h		\
	opt_stack.h		\
	opt_swap.h		\
	opt_syscons.h		\
	opt_tcpdebug.h	\
	opt_timer.h	\
	opt_turnstile_profiling.h	\
	opt_umtx_profiling.h		\
	opt_vm.h		\
	opt_vfs_aio.h	\
	opt_watchdog.h	\
	opt_witness.h	\
	opt_zero.h

# Search for kernel source tree in standard places.
.for _dir in ${.CURDIR}/../.. ${.CURDIR}/../../sys ${.CURDIR}/../../.. /sys /usr/src/sys
.if !defined(SYSDIR) && exists(${_dir}/kern/)
SYSDIR=	${_dir}
.endif
.endfor
.if !defined(SYSDIR) || !exists(${SYSDIR}/kern/)
.error "can't find kernel source tree"
.endif


PREFIX= ${SYSDIR}
.PATH: ${PREFIX}/crypto
.PATH: ${PREFIX}/kern

.if ${MACHINE_CPUARCH} == "i386" 
.PATH: arch/x86/x86
.PATH: ${PREFIX}/i386/i386
.PATH: ${PREFIX}/x86/x86
.endif 

.if ${MACHINE_CPUARCH} == "amd64"
.PATH: arch/amd64/amd64
.PATH: arch/amd64/pci
.PATH: arch/x86/x86
.PATH: ${PREFIX}/amd64/amd64
.PATH: ${PREFIX}/amd64/acpica
.PATH: ${PREFIX}/amd64/pci
.PATH: ${PREFIX}/x86/x86
.PATH: ${PREFIX}/x86/pci
.PATH: ${PREFIX}/x86/acpica
PN_PCI_OBJS += ${PN_X86_PCI_OBJS}
PN_ACPICA_OBJS += ${PN_X86_ACPICA_OBJS}
CFLAGS+=	-DHAMMER
.endif

.PATH: ${PREFIX}/ddb
.PATH: ${PREFIX}/net
.PATH: ${PREFIX}/netinet
.PATH: ${PREFIX}/netinet6
.PATH: ${PREFIX}/netinet/cc
.PATH: ${PREFIX}/vm
.PATH: ${PREFIX}/geom
.PATH: ${PREFIX}/fs/deadfs
.PATH: ${PREFIX}/fs/devfs
.PATH: ${PREFIX}/libkern
.PATH: ${PREFIX}/dev/random
.PATH: ${PREFIX}/dev/watchdog
.PATH: ${PREFIX}/dev/led
.PATH: dev/pci
.PATH: ${PREFIX}/dev/pci
.PATH: ${PREFIX}/dev/e1000
.PATH: ${PREFIX}/contrib/dev/acpica/components/dispatcher
.PATH: ${PREFIX}/contrib/dev/acpica/components/events
.PATH: ${PREFIX}/contrib/dev/acpica/components/executer
.PATH: ${PREFIX}/contrib/dev/acpica/components/hardware
.PATH: ${PREFIX}/contrib/dev/acpica/components/namespace
.PATH: ${PREFIX}/contrib/dev/acpica/components/parser
.PATH: ${PREFIX}/contrib/dev/acpica/components/resources
.PATH: ${PREFIX}/contrib/dev/acpica/components/tables
.PATH: ${PREFIX}/contrib/dev/acpica/components/utilities
.PATH: ${PREFIX}/dev/acpica
.PATH: ${PREFIX}/dev/acpica/Osd
.PATH: drivers
.PATH: ukern_sys

PN_COMMON_OBJS =				\
	${PN_SYS_COMMON_OBJS}		\
	${PN_DDB_COMMON_OBJS}		\
	${PN_KERN_COMMON_OBJS}		\
	${PN_LIBKERN_COMMON_OBJS}	\
	${PN_FS_COMMON_OBJS}		\
	${PN_NET_COMMON_OBJS}		\
	${PN_VM_COMMON_OBJS}		\
	${PN_MD_COMMON_OBJS}		\
	${PN_PCI_OBJS}				\
	${PN_EM_OBJS}				\
	${PN_OTHER_COMMON_OBJS}

#	${PN_ACPICA_OBJS}			\
#	${PN_NETINET_COMMON_OBJS}	\
#	${PN_NETINET6_COMMON_OBJS}	\


PN_COMMON_SRCS= ${PN_MD_ASM_OBJS:C/.o$/.S/}
PN_COMMON_SRCS+= ${PN_COMMON_OBJS:C/.o$/.c/}

SRCS=	${PN_COMMON_SRCS}
SRCS+=	${PN_MFILES}
SRCS+=	${PN_OPT_FILES}
SRCS+=	${GEN_CFILES}


CFLAGS+=	-I./include
CFLAGS+=	-I./include/opt
CFLAGS+=	-I${PREFIX}
CFLAGS+=	-I.
CFLAGS+=	-I${PREFIX}/contrib/altq
CFLAGS+=	-I${PREFIX}/contrib/pf
CFLAGS+=	-I/usr/include
CFLAGS+=	-D_KERNEL
CFLAGS+=	-DPLEBNET
CFLAGS+=	-DEARLY_PRINTF
CFLAGS+=	-DMAXUSERS=32
CFLAGS+=	-DKDB
CFLAGS+=	-DDDB
CFLAGS+=	-DSMP
CFLAGS+=	-DVERBOSE_SYSINIT
CFLAGS+=	-DBOOTVERBOSE
CFLAGS+=	-DVM_KMEM_SIZE=100*1024*1024
CFLAGS+=	-DINVARIANT_SUPPORT
CFLAGS+=	-DINVARIANTS
#CFLAGS+=	-DINET=1
#CFLAGS+=	-DINET6=1
CFLAGS+=	-fformat-extensions
CFLAGS+=	-fno-builtin
CFLAGS+=	-fPIC
CFLAGS+=	-g -O0
CFLAGS+=	-nostdinc
NO_WERROR=	1
WARNS=2
CSTD="c99"
WFORMAT=0



AWK ?= awk

_ILINKS=@ machine 
MACHINE_CPUARCH ?= ${MACHINE}
.if ${MACHINE} != ${MACHINE_CPUARCH}
_ILINKS+=${MACHINE_CPUARCH}
.endif

# newer versions of the kernel ...
.if ${MACHINE_CPUARCH} == "i386" || ${MACHINE_CPUARCH} == "amd64"
_ILINKS+=x86
.endif


all: ${_ILINKS}

beforedepend: ${_ILINKS}

# Ensure that the links exist without depending on it when it exists which
# causes all the modules to be rebuilt when the directory pointed to changes.
.for _link in ${_ILINKS}
.if !exists(${.OBJDIR}/${_link})
${OBJS}: ${_link}
.endif
.endfor

${_ILINKS}:
	@case ${.TARGET} in \
	machine) \
		path=${SYSDIR}/${MACHINE}/include ;; \
	@) \
		path=${SYSDIR} ;; \
	*) \
		path=${SYSDIR}/${.TARGET}/include ;; \
	esac ; \
	path=`(cd $$path && /bin/pwd)` ; \
	${ECHO} ${.TARGET} "->" $$path ; \
	ln -sf $$path ${.TARGET}


.if defined(KERNBUILDDIR)
.PATH: ${KERNBUILDDIR}
CFLAGS+=	-I${KERNBUILDDIR}
.for _src in ${SRCS:Mopt_*.h}
CLEANFILES+=	${_src}
.if !target(${_src})
${_src}:
	ln -sf ${KERNBUILDDIR}/${_src} ${.TARGET}
.endif
.endfor
.else
.for _src in ${SRCS:Mopt_*.h}
CLEANFILES+=	${_src}
.if !target(${_src})
${_src}:
	:> ${.TARGET}
.endif
.endfor
.endif


vers.c: @/conf/newvers.sh @/sys/param.h ${SYSTEM_DEP}
	MAKE=${MAKE} sh @/conf/newvers.sh ${KERN_IDENT}

vnode_if.c: @/tools/vnode_if.awk @/kern/vnode_if.src
	${AWK} -f @/tools/vnode_if.awk @/kern/vnode_if.src -c

vnode_if.h vnode_if_newproto.h vnode_if_typedef.h: @/tools/vnode_if.awk \
    @/kern/vnode_if.src

MFILES= kern/bus_if.m kern/device_if.m kern/linker_if.m kern/cpufreq_if.m	\
	kern/clock_if.m dev/pci/pci_if.m dev/pci/pcib_if.m isa/isa_if.m 		\
	dev/acpica/acpi_if.m dev/acpi_support/acpi_wmi_if.m

GEN_CFILES= ${MFILES:T:S/.m$/.c/}


.if !empty(SRCS:Mvnode_if.h)
CLEANFILES+=	vnode_if.h vnode_if_newproto.h vnode_if_typedef.h
.if !exists(@)
vnode_if.h vnode_if_newproto.h vnode_if_typedef.h: @
.else
vnode_if.h vnode_if_newproto.h vnode_if_typedef.h: @/tools/vnode_if.awk \
    @/kern/vnode_if.src
.endif
vnode_if.h: vnode_if_newproto.h vnode_if_typedef.h
	${AWK} -f @/tools/vnode_if.awk @/kern/vnode_if.src -h
vnode_if_newproto.h:
	${AWK} -f @/tools/vnode_if.awk @/kern/vnode_if.src -p
vnode_if_typedef.h:
	${AWK} -f @/tools/vnode_if.awk @/kern/vnode_if.src -q
.endif

.for _srcsrc in ${MFILES}
.for _ext in c h
.for _src in ${SRCS:M${_srcsrc:T:R}.${_ext}}
CLEANFILES+=	${_src}
.if !target(${_src})
.if !exists(@)
${_src}: @
.else
${_src}: @/tools/makeobjops.awk @/${_srcsrc}
.endif
	${AWK} -f @/tools/makeobjops.awk @/${_srcsrc} -${_ext}
.endif
.endfor # _src
.endfor # _ext
.endfor # _srcsrc


.include <bsd.dep.mk>

cleandepend: cleanilinks

cleanilinks:
	rm -f ${_ILINKS}

.if !exists(${.OBJDIR}/${DEPENDFILE})
${OBJS}: ${SRCS:M*.h}
.endif

.include <bsd.lib.mk>
