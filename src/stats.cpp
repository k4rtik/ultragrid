#include "stats.h"

#include <map>
#include <string>

#include "compat/platform_spin.h"
#include "debug.h"
#include "messaging.h"
#include "module.h"
#include "utils/lock_guard.h"
#include "utils/resource_manager.h"

#define MAX_ITEMS 100

using namespace std;

static struct response *messaging_callback(struct received_message *msg, void *udata);

struct stats {
        public:
                stats(string name) {
                        m_name = name;
                        m_val = 0;
                }

                ~stats() {
                }

                void update_int(int64_t val) {
                        m_val = val;
                }

                struct response *process_message(void *msg) {
                        struct msg_stats *stats = (struct msg_stats *) msg;

                        if(string(stats->what).compare(m_name) == 0) {
                                char buf[128];
                                snprintf(buf, sizeof(buf), "%lu", m_val);
                                return new_response(RESPONSE_OK, strdup(buf));
                        } else {
                                return NULL;
                        }
                }

        private:
                string m_name;
                int64_t m_val;
                void *m_messaging_subscribtion;
};

static struct response *messaging_callback(void *msg, struct module *mod)
{
        stats *s = (stats *) mod->priv_data;
        return s->process_message(msg);
}

struct stats *stats_new_statistics(char * name)
{
        return new stats(string(name));
}

void stats_update_int(struct stats *s, int64_t val)
{
        return s->update_int(val);
}

void stats_destroy(struct stats *s)
{
        delete s;
}
