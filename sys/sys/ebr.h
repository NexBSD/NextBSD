#ifndef _SYS_EBR_H_
#define _SYS_EBR_H_

struct ebr_epoch;
struct ebr_entry;

typedef struct ebr_epoch *ebr_epoch_t;
typedef struct ebr_entry *ebr_entry_t;

typedef void (*ebr_callback_t)(void *cookie);


ebr_epoch_t ebr_epoch_alloc(int pcpu_count);
ebr_entry_t ebr_epoch_entry_alloc(int flags);
void ebr_epoch_entry_free(ebr_entry_t entry);

int ebr_epoch_entry_init(ebr_epoch_t eepoch, ebr_entry_t entry, void *cookie, bool cansleep);
void *ebr_epoch_read_lock(ebr_epoch_t ee);
void ebr_epoch_read_unlock(void *cookie);
void ebr_epoch_defer(ebr_epoch_t epoch, ebr_entry_t entry, ebr_callback_t fn);
void ebr_epoch_synchronize(ebr_epoch_t eepoch);

#endif
