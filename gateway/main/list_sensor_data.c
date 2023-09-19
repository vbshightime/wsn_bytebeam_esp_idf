// LinkedList.c

#include <stdio.h>
#include <stdlib.h>

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

// Linked list node.
struct Node {
  struct SensorPayload payload;
  const char* mac_str;
  struct Node* next;
};

// Function to add a node to the linked list.
void addNode(struct Node **head, struct SensorPayload payload, const char* mac_str) {
  // Create a new node.
  struct Node *newNode = malloc(sizeof(struct Node));

  // Copy the data to the new node.
  newNode->payload = payload;

  newNode->mac_str = mac_str;

  // Set the next node to NULL.
  newNode->next = NULL;

  // If the linked list is empty, make the new node the head.
  if (*head == NULL) {
    *head = newNode;
  } else {
    // Otherwise, find the last node in the linked list and add the new node after it.
    struct Node *lastNode = *head;
    while (lastNode->next != NULL) {
      lastNode = lastNode->next;
    }
    lastNode->next = newNode;
  }
}

// Function to get the number of nodes in the linked list.
int countNodes(struct Node *head) {
  int count = 0;
  struct Node *node = head;
  while (node != NULL) {
    count++;
    node = node->next;
  }
  return count;
}

struct Node* readNode(struct Node* head, int index) {
  if (head == NULL) {
    return NULL;
  }

  struct Node* currentNode = head;
  for (int i = 0; i < index; i++) {
    if (currentNode->next == NULL) {
      return NULL;
    }

    currentNode = currentNode->next;
  }

  return currentNode;
}
