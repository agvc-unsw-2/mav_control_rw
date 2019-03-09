mav_control_interface
==================

Basic interface that provides ros access to the (position) controller, and takes input from the RC into account. 

Check function RcInterfaceAci::rcCallback for details on what different buttons do

E.g.
button[0]: hold on to enable RC
axes[4]: switches RC modes
axes[5]: turns control interface on and off (disable input if off)


  
#params
* `param<std::string>("test", test, "test");`
  

