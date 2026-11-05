#pragma once
#include <string>
#include <functional>
#include <mosquitto.h>
using namespace std;

class MqttClient {
public:
    MqttClient(const string& clientId, const string& host = "localhost", int port = 1883);
    ~MqttClient();

    bool connect();
    void disconnect();

    bool publish(const string& topic, const string& payload, int qos = 0, bool retain = false);
    bool subscribe(const string& topic, int qos = 0);

    void setMessageCallback(function<void(const string& topic, const string& payload)> cb);

private:
    struct mosquitto* mosq;
    string clientId;
    string host;
    int port;
    function<void(const string&, const string&)> messageCallback;

    static void onMessageStatic(struct mosquitto* m, void* userdata, const struct mosquitto_message* msg);
    static void onConnectStatic(struct mosquitto* m, void* userdata, int rc);
};