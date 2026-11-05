#include "Arduino.h"
#define HTTP_GET 0
#define HTTP_POST 1
struct WebServer{
  std::map<std::string,String> args,headers;
  std::map<std::string,std::function<void()>> routes;
  int code=0; String response;
  WebServer(int){}
  bool hasArg(const char *name){return args.count(name)!=0;}
  String arg(const char *name){return args[name];}
  String header(const char *name){return headers[name];}
  void send(int value,const char *,const String &data){code=value;response=data;}
  void send_P(int value,const char *type,const char *data){send(value,type,data);}
  void sendHeader(const char *,const char *){}
  void collectHeaders(const char **,int){}
  void on(const char *path,int,std::function<void()> f){routes[path]=f;}
  void onNotFound(std::function<void()>){ }
  void begin(){} void handleClient(){}
};
