// LinkedList.h

#ifndef LINKED_LIST_H
#define LINKED_LIST_H

// Structure to store the data from the sensor node.
struct SensorPayload {
    uint8_t sensor_profile;           // identifies sensor profile
    uint8_t hwRev;                  // identifies hw revision
    uint8_t fwRev;                 // identifies fw revision
    uint8_t deviceType;           // identifies device type            
    uint8_t batteryPercentage;  //batteryPercentage of nodes
    float temperature;
    float humidity;
}__attribute__ ((packed));

struct Node {
  struct SensorPayload payload;
  const char* mac_str;
  struct Node* next;
};

void addNode(struct Node **head, struct SensorPayload payload);
int countNodes(struct Node *head);
struct Node* readNode(struct Node* head, int index);

#endif