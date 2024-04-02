#include <LinkedList.h>
#include "maze.h"


// Define a queue using LinkedList to store maze elements (Cell type)
LinkedList<Cell> Queue;


void enqueue(struct Cell &item) {
  Queue.add(item);
}


bool isEmpty() {
  return Queue.size() == 0;
}

Cell dequeue() {
  Cell emptyCell; 
  if (!isEmpty()) {
    Cell item = Queue.get(0);
    Queue.remove(0);
    return item;
  } else {
    Serial.println("Queue is empty!");
    return emptyCell; 
  }
}

Cell front() {
  Cell emptyCell; 
  if (!isEmpty()) {
    return Queue.get(0);
  } else {
    Serial.println("Queue is empty!");
    return emptyCell; 
  }
}

