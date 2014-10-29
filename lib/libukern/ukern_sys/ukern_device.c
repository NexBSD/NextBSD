#include <sys/param.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/pci_pass.h>
#include <sys/libkern.h>
#include <ukern_device.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>


int *__error();
extern int ioctl(int fd, unsigned long request, ...);


int
device_get_fd(device_t dev)
{
	char devstr[24];
	char *fdstr;

	snprintf(devstr, sizeof(devstr), "/dev/pci%02x:%02x:%02x:%02x",
			 pci_get_domain(dev), pci_get_bus(dev), pci_get_slot(dev),
			 pci_get_function(dev));

	if ((fdstr = kern_getenv(devstr)) == NULL)
		return (-1);
	return (strtol(fdstr, NULL, 0));
}

int
device_get_irq(device_t dev, int vector)
{
	int fd;

	fd = device_get_fd(dev);
	if (ioctl(fd, DEVPASSIOCCHKIRQ, &vector))
		return (*__error());
	/* XXX register intr source */
	
	return (0);
}
