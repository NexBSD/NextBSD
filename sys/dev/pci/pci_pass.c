/*
 * 'filter' driver - like ignore - eats up devices, but in this case
 * ones that are specified as tunables to permit FreeBSD to serve as
 * a ukern host
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/libkern.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/rwlock.h>
#include <dev/pci/pcivar.h>


#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_map.h>
#include <vm/vm_param.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>

static int	unit;
static int	pci_pass_probe(device_t dev);
static int	pci_pass_attach(device_t dev);
static device_method_t pci_pass_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,		pci_pass_probe),
    DEVMETHOD(device_attach,	pci_pass_attach),
    { 0, 0 }
};

struct pci_pass_softc {
	device_t pci_dev;
	struct cdev *pci_cdev;
};

static driver_t pci_pass_driver = {
    "pci_pass",
    pci_pass_methods,
    sizeof(struct pci_pass_softc),
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


struct pci_pass_handle {
	struct cdev *cdev;
	size_t		size;
	vm_paddr_t	paddr;
};

static int
pci_pass_dev_pager_ctor(void *handle, vm_ooffset_t size, vm_prot_t prot,
    vm_ooffset_t foff, struct ucred *cred, u_short *color)
{
	struct pci_pass_handle *pph = handle;

	dev_ref(pph->cdev);
	return (0);
}

static void
pci_pass_dev_pager_dtor(void *handle)
{
	struct pci_pass_handle *pph = handle;
	struct cdev *cdev = pph->cdev;

	free(pph, M_DEVBUF);
	dev_rel(cdev);
}

static int
pci_pass_dev_pager_fault(vm_object_t object, vm_ooffset_t offset,
	int prot, vm_page_t *mres)
{
	struct pci_pass_handle *pph = object->handle;
	vm_paddr_t paddr;
	vm_page_t page;
	vm_memattr_t memattr;
	vm_pindex_t pidx;

	printf("pci_pass_dev_pager_fault: object %p offset %jx prot %d mres %p",
			object, (intmax_t)offset, prot, mres);

	if (offset < pph->paddr || offset > pph->paddr + pph->size)
		return (VM_PAGER_FAIL);

	memattr = object->memattr;
	paddr = offset;
	pidx = OFF_TO_IDX(offset);

	if (((*mres)->flags & PG_FICTITIOUS) != 0) {
		/*
		 * If the passed in result page is a fake page, update it with
		 * the new physical address.
		 */
		page = *mres;
		vm_page_updatefake(page, paddr, memattr);
	} else {
		/*
		 * Replace the passed in reqpage page with our own fake page and
		 * free up the all of the original pages.
		 */
#ifndef VM_OBJECT_WUNLOCK	/* FreeBSD < 10.x */
#define VM_OBJECT_WUNLOCK VM_OBJECT_UNLOCK
#define VM_OBJECT_WLOCK	VM_OBJECT_LOCK
#endif /* VM_OBJECT_WUNLOCK */

		VM_OBJECT_WUNLOCK(object);
		page = vm_page_getfake(paddr, memattr);
		VM_OBJECT_WLOCK(object);
		vm_page_lock(*mres);
		vm_page_free(*mres);
		vm_page_unlock(*mres);
		*mres = page;
		vm_page_insert(page, object, pidx);
	}
	page->valid = VM_PAGE_BITS_ALL;
	return (VM_PAGER_OK);
}


static struct cdev_pager_ops pci_pass_cdev_pager_ops = {
	.cdev_pg_ctor = pci_pass_dev_pager_ctor,
	.cdev_pg_dtor = pci_pass_dev_pager_dtor,
	.cdev_pg_fault = pci_pass_dev_pager_fault,
};

/*
 * XXX super crude proof of concept
 */
static int
pci_pass_mmap_single(struct cdev *cdev, vm_ooffset_t *offset,
		   vm_size_t size, struct vm_object **objp, int nprot)
{
	struct vm_object *obj;
	struct pci_pass_handle *pph;
	
	printf("pci_pass_mmap_single(%p, %p=%lx,%ld, %p, %d)\n",
		   cdev, offset, *offset, size, objp, nprot);

	pph = malloc(sizeof(struct pci_pass_handle), M_DEVBUF, M_NOWAIT|M_ZERO);
	if (pph == NULL)
		return (ENOMEM);
	pph->cdev = cdev;
	pph->size = size;
	/* XXX validate that this address actually belongs to this device */
	pph->paddr = (vm_paddr_t)*offset;
	obj = cdev_pager_allocate(pph, OBJT_DEVICE, &pci_pass_cdev_pager_ops, size,
							  nprot, *offset, NULL);
	if (obj == NULL) {
		free(pph, M_DEVBUF);
		return (ENXIO);
	}
	*objp = obj;
	return (0);
}
