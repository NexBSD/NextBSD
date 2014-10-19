/*
 * 'filter' driver - like ignore - eats up devices, but in this case
 * ones that are specified as tunables to permit FreeBSD to serve as
 * a ukern host
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/libkern.h>

#include <dev/pci/pcivar.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>

static int	unit;
static int	pci_pass_probe(device_t dev);
static int	pci_pass_attach(device_t dev);
static device_method_t pci_pass_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,		pci_pass_probe),
    DEVMETHOD(device_attach,	pci_pass_attach),
    { 0, 0 }
};

#define MAX_MM_OBJS 10
struct memory_mappings {
	vm_ooffset_t offset;
	vm_size_t size;
	struct vm_object *object;
};
struct pci_pass_softc {
	device_t pci_dev;
	struct cdev *pci_cdev;
	int obj_count;
	struct memory_mappings mm_objs[MAX_MM_OBJS];
};

static driver_t pci_pass_driver = {
    "pci_pass",
    pci_pass_methods,
    sizeof(struct pci_pass_softc),
};

static d_close_t pci_pass_close;
static d_open_t pci_pass_open;
static d_mmap_single_t pci_pass_mmap_single;

static struct cdevsw pci_pass_cdevsw = {
       .d_version =    D_VERSION,
       .d_flags =      0,
       .d_open =       pci_pass_open,
       .d_close =      pci_pass_close,
       .d_mmap_single =      pci_pass_mmap_single,
       .d_name =       "pcipass",
};
static devclass_t pci_pass_devclass;

DRIVER_MODULE(pci_pass, pci, pci_pass_driver, pci_pass_devclass, 0, 0);

static int
pci_pass_probe(device_t dev)
{
	char devstr[24];
	char filter_list[128], *filterstr;
	char *curdevid;

	snprintf(devstr, sizeof(devstr), "%02x:%02x:%02x:%02x", pci_get_domain(dev),
			 pci_get_bus(dev),pci_get_slot(dev), pci_get_function(dev));

	if (getenv_string("device_filter_list", filter_list, sizeof(filter_list)) == 0)
		return (ENXIO);

	filterstr = filter_list;
	while ((curdevid = strsep(&filterstr, ",")) != NULL)
		if (strncmp(devstr, curdevid, sizeof(devstr)) == 0)
			return (BUS_PROBE_SPECIFIC);

    return(ENXIO);
}

static int
pci_pass_attach(device_t dev)
{
	struct pci_pass_softc *sc = device_get_softc(dev);
	sc->pci_dev = dev;
	sc->pci_cdev =
		make_dev(&pci_pass_cdevsw, unit, UID_ROOT, GID_WHEEL, 0660,
				 "pci%02x:%02x:%02x:%02x", pci_get_domain(dev),
				 pci_get_bus(dev),pci_get_slot(dev), pci_get_function(dev));
					
	if (sc->pci_cdev == NULL)
		return (ENOMEM);

	device_set_desc(dev, "filter");		
	sc->pci_cdev->si_drv1 = (void *)sc;
	sc->obj_count = 0;
	return (0);
}

static int
pci_pass_open(struct cdev *dev, int flags, int fmp, struct thread *td)
{
	printf("pci_pass_open(...) called\n");
	return (0);
}

static int
pci_pass_close(struct cdev *dev, int flags, int fmt, struct thread *td)
{
	printf("pci_pass_close(...) called\n");
	return (0);
}

/*
 * XXX super crude proof of concept
 */
static int
pci_pass_mmap_single(struct cdev *cdev, vm_ooffset_t *offset,
		   vm_size_t size, struct vm_object **object, int nprot)
{
	struct pci_pass_softc *sc = cdev->si_drv1;
#ifdef notyet	
	device_t pci_dev = sc->pci_dev;
#endif	
	struct vm_object *obj;
	int i;

	printf("pci_pass_mmap_single(%p, %p=%lx,%ld, %p, %d)\n",
		   cdev, offset, *offset, size, object, nprot);
	for (i = 0; i < sc->obj_count; i++) {
		if (*offset >= sc->mm_objs[i].offset && *offset < sc->mm_objs[i].offset + size &&
			sc->mm_objs[i].size <= size) {
			*object = sc->mm_objs[i].object;
			break;
		}
	}
	if (i < MAX_MM_OBJS) {
		if ((obj= vm_object_allocate(OBJT_DEVICE, size)) == NULL)
			return (ENXIO);

		sc->mm_objs[i].offset = *offset;
		sc->mm_objs[i].size = size;
		*object = sc->mm_objs[i].object = obj;
		i++;
		return (0);
	} 
	return (ENXIO);
}
