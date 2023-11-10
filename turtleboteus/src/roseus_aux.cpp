#include <setjmp.h>
#include <boost/thread/mutex.hpp>  // eus_throw require this

#include <ros/ros.h>
#include <console_bridge/console.h>

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string

#include "eus.h"

extern "C" {
  pointer ___roseus_aux(register context *ctx, int n, pointer *argv, pointer env);
  void register_roseus_aux(){
    char modname[] = "___roseus_aux";
    return add_module_initializer(modname, (pointer (*)())___roseus_aux);}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string

//using namespace ros;

pointer QCONSOLE_BRIDGE_LOG_DEBUG,QCONSOLE_BRIDGE_LOG_INFO,QCONSOLE_BRIDGE_LOG_WARN,QCONSOLE_BRIDGE_LOG_ERROR;

/************************************************************
 *   rosconsole_bridge
 ************************************************************/
#define def_console_bridge_formatter(funcname, rosfuncname)     \
  pointer funcname(register context *ctx,int n,pointer *argv)   \
  { pointer *argv2,msg;                                         \
    int argc2;                                                  \
    argc2 = n+1;                                                \
    argv2 = (pointer *)malloc(sizeof(pointer)*argc2);           \
    argv2[0] = NIL;                                             \
    for(int i=0;i<n;i++) argv2[i+1]=argv[i] ;                   \
    msg = XFORMAT(ctx, argc2, argv2);                           \
    rosfuncname("%s", msg->c.str.chars);                        \
    free(argv2);                                                \
    return (T);                                                 \
  }

def_console_bridge_formatter(ROSEUS_CONSOLE_BRIDGE_LOGDEBUG, CONSOLE_BRIDGE_logDebug)
def_console_bridge_formatter(ROSEUS_CONSOLE_BRIDGE_LOGINFO,  CONSOLE_BRIDGE_logInform)
def_console_bridge_formatter(ROSEUS_CONSOLE_BRIDGE_LOGWARN,  CONSOLE_BRIDGE_logWarn)
def_console_bridge_formatter(ROSEUS_CONSOLE_BRIDGE_LOGERROR, CONSOLE_BRIDGE_logError)

pointer ROSEUS_CONSOLE_BRIDGE_SET_LOG_LEVEL(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  int log_level = ckintval(argv[0]);
  console_bridge::LogLevel level;
  switch(log_level){
  case 0:
    level = console_bridge::CONSOLE_BRIDGE_LOG_DEBUG;
    break;
  case 1:
    level = console_bridge::CONSOLE_BRIDGE_LOG_INFO;
    break;
  case 2:
    level = console_bridge::CONSOLE_BRIDGE_LOG_WARN;
    break;
  case 3:
    level = console_bridge::CONSOLE_BRIDGE_LOG_ERROR;
  default:
    return (NIL);
  }

  console_bridge::setLogLevel(level);
  return (T);
}

pointer ROSEUS_CONSOLE_BRIDGE_GET_LOG_LEVEL(register context *ctx,int n,pointer *argv)
{
  console_bridge::LogLevel level = console_bridge::getLogLevel();
  return makeint(level);
}

pointer ___roseus_aux(register context *ctx, int n, pointer *argv, pointer env)
{
  pointer rospkg,p=Spevalof(PACKAGE);
  rospkg=findpkg(makestring("ROS",3));
  if (rospkg == 0 ) {
    ROS_ERROR("Coudld not found ROS package; Please load eusros.so");
    exit(2);
  }
  Spevalof(PACKAGE)=rospkg;

  QCONSOLE_BRIDGE_LOG_DEBUG=defvar(ctx,"*CONSOLE-BRIDGE-LOG-DEBUG*",makeint(0),rospkg);
  QCONSOLE_BRIDGE_LOG_INFO=defvar(ctx,"*CONSOLE-BRIDGE-LOG-INFO*",makeint(1),rospkg);
  QCONSOLE_BRIDGE_LOG_WARN=defvar(ctx,"*CONSOLE-BRIDGE-LOG-WARN*",makeint(2),rospkg);
  QCONSOLE_BRIDGE_LOG_ERROR=defvar(ctx,"*CONSOLE-BRIDGE-LOG-ERROR*",makeint(3),rospkg);
  defun(ctx,"CONSOLE-BRIDGE-SET-LOG-LEVEL",argv[0],(pointer (*)())ROSEUS_CONSOLE_BRIDGE_SET_LOG_LEVEL, "set the minimum level of logging data to output to console_bridge");
  defun(ctx,"CONSOLE-BRIDGE-GET-LOG-LEVEL",argv[0],(pointer (*)())ROSEUS_CONSOLE_BRIDGE_GET_LOG_LEVEL, "retrieve the current level of console_bridge logging data");
  defun(ctx,"CONSOLE-BRIDGE-LOG-DEBUG",argv[0],(pointer (*)())ROSEUS_CONSOLE_BRIDGE_LOGDEBUG, "debug level logging function of console_bridge");
  defun(ctx,"CONSOLE-BRIDGE-LOG-INFO",argv[0],(pointer (*)())ROSEUS_CONSOLE_BRIDGE_LOGINFO, "info level logging function of console_bridge");
  defun(ctx,"CONSOLE-BRIDGE-LOG-WARN",argv[0],(pointer (*)())ROSEUS_CONSOLE_BRIDGE_LOGWARN, "warn level logging function of console_bridge");
  defun(ctx,"CONSOLE-BRIDGE-LOG-ERROR",argv[0],(pointer (*)())ROSEUS_CONSOLE_BRIDGE_LOGDEBUG, "error level logging function of console_bridge");

  return 0;
}
