#include <boost/interprocess/ipc/message_queue.hpp>

using namespace boost::interprocess;

int main()
{
    // Shut down properly
    message_queue mq
            (open_only,
             "etree_message_queue");
    bool shouldQuit = true;
    mq.send(&shouldQuit, sizeof(shouldQuit), 0);
}
