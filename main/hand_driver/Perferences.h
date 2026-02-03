extern "C"{
    #include "nvs_flash.h"
}
class Preferences{
private:
    nvs_handle_t handle;
    bool closed=true;
public:
    Preferences();
    void begin(const char *ns,bool rw);//true readonly
    void putInt(const char *key,int valuse);
    int getInt(const char *key,int valuse);
    void end();
};