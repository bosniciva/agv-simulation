#include "mqttclient.h"
#include <iostream>
using namespace std;

MqttClient::MqttClient(const string& clientId_, const string& host_, int port_)
    : mosq(nullptr), clientId(clientId_), host(host_), port(port_) {
    mosquitto_lib_init();
    mosq = mosquitto_new(clientId.c_str(), true, this);
    if (mosq) {
        mosquitto_message_callback_set(mosq, onMessageStatic);
        mosquitto_connect_callback_set(mosq, onConnectStatic);
    }
    else {
        cerr << "[MQTT] mosquitto_new failed." << endl;
    }
}

MqttClient::~MqttClient() {
    if (mosq) {
        mosquitto_loop_stop(mosq, true);
        mosquitto_destroy(mosq);
    }
    mosquitto_lib_cleanup();
}

bool MqttClient::connect() {
    if (!mosq) return false;

    int rc = mosquitto_connect(mosq, host.c_str(), port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        cerr << "[MQTT] Connection failed, error code: " << rc << endl;
        return false;
    }

    mosquitto_loop_start(mosq);
    return true;
}

void MqttClient::disconnect() {
    if (mosq) {
        mosquitto_disconnect(mosq);
        mosquitto_loop_stop(mosq, true);
    }
}

bool MqttClient::publish(const string& topic, const string& payload, int qos, bool retain) {
    if (!mosq) return false;
    int rc = mosquitto_publish(mosq, nullptr, topic.c_str(),
        static_cast<int>(payload.size()), payload.c_str(), qos, retain);
    return rc == MOSQ_ERR_SUCCESS;
}

bool MqttClient::subscribe(const string& topic, int qos) {
    if (!mosq) return false;
    int rc = mosquitto_subscribe(mosq, nullptr, topic.c_str(), qos);
    return rc == MOSQ_ERR_SUCCESS;
}

void MqttClient::setMessageCallback(function<void(const string&, const string&)> cb) {
    messageCallback = cb;
}

void MqttClient::onConnectStatic(struct mosquitto* m, void* userdata, int rc) {
    (void)m;
    (void)userdata;
    if (rc == 0) {
        cout << "[MQTT] Connected to broker." << endl;
    }
    else {
        cerr << "[MQTT] Connection rejected, code: " << rc << endl;
    }
}

void MqttClient::onMessageStatic(struct mosquitto* m, void* userdata, const struct mosquitto_message* msg) {
    (void)m;
    MqttClient* self = static_cast<MqttClient*>(userdata);
    if (self && self->messageCallback && msg->payload) {
        string topic(msg->topic);
        string payload(static_cast<char*>(msg->payload), msg->payloadlen);
        self->messageCallback(topic, payload);
    }
}