# $FreeBSD$
# $NextBSD$

SCRIPTS = mach-tests-setup
CLEANFILES = mach-tests-setup mach-tests-setup.sh
NEWVERS = ${.CURDIR}/../../../sys/conf/newvers.sh
BINDIR?= /usr/bin

mach-tests-setup.sh: ${.CURDIR}/mach-tests-setup.sh.in ${NEWVERS}
	eval $$(egrep '^(TYPE|REVISION|BRANCH)=' ${NEWVERS}) ; \
	if [ "$${TYPE}" != "NextBSD" ] ; then \
		TYPE="NextBSD" ; \
	fi ; \
	if ! sed -e "\
			s/@@TYPE@@/$${TYPE}/g; \
			s/@@REVISION@@/$${REVISION}/g; \
			s/@@BRANCH@@/$${BRANCH}/g; \
			s/@@COPYRIGHT@@/$${COPYRIGHT}/g; \
		" ${.CURDIR}/mach-tests-setup.sh.in >${.TARGET} ; then \
		rm -f ${.TARGET} ; \
		exit 1 ; \
	fi

.include <bsd.prog.mk>
