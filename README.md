
Model Predictive Controller
===========================

This is my somewhat incorrect, but working implementation of a model predictive
controller. It's done this way because it seems to match better the physics of
the simulator, the non-linear relationship between acceleration and throttle,
and the actual delays in the system.

[![Model Predictive Controller](https://img.youtube.com/vi/Ro-pCzWNXCc/0.jpg)](https://www.youtube.com/watch?v=Ro-pCzWNXCc "Particle filter")

Building
--------

The program needs:

 * IPOPT and CppAD ([https://projects.coin-or.org/Ipopt](https://projects.coin-or.org/Ipopt))
 * uWebSockets ([https://github.com/uNetworking/uWebSockets](https://github.com/uNetworking/uWebSockets))
 * The NLohmann's JSON library ([https://github.com/nlohmann/json](https://github.com/nlohmann/json))

If they are installed in system directories, they should be picked up
automatically. Otherwise use `UWEBSOCKETS_ROOTDIR` and `NLOHMANNJSON_ROOT_DIR`.
On MacOS, you may need to set `OPENSSL_ROOT_DIR` as well.
