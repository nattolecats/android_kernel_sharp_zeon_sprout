#ifndef _TOUCHEVENT_NOTIFIER_H_
#define _TOUCHEVENT_NOTIFIER_H_

#include <linux/notifier.h>

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM)
int touchevent_register_client(struct notifier_block *nb);
int touchevent_unregister_client(struct notifier_block *nb);

#else
int touchevent_register_client(struct notifier_block *nb){ return 0; }
int touchevent_unregister_client(struct notifier_block *nb){ return 0; }

#endif

#endif /* _TOUCHEVENT_NOTIFIER_H_ */
