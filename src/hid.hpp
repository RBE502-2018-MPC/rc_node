extern "C" int rawhid_open(int max, int vid, int pid, int usage_page, int usage);
extern "C" int rawhid_recv(int num, void *buf, int len, int timeout);
extern "C" int rawhid_send(int num, void *buf, int len, int timeout);
extern "C" void rawhid_close(int num);
