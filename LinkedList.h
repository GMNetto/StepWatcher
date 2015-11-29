typedef struct node{
  int value;
  struct node* next;
}Node;

class LinkedList{
private:
  Node *head, *tail;
  int size_l;
  int maximum_size;
public:
  LinkedList(int maximum_size){
    this->maximum_size = maximum_size;
    head = 0;
    tail = 0;
  }

  int insert(int value){
    Node *new_node = new Node();
    new_node->value = value;
    if(!head){
      head = new_node;
      tail = new_node;
      size_l = 1;
      return -1;
    }
    if(size_l >= maximum_size){
      Node* old_head = this->head;
      this->head = this->head->next;
      delete(old_head);
      tail->next = new_node;
      tail = new_node;
      return -2;
    }
    tail->next = new_node;
    tail = new_node;
    size_l++;
    return int(head->next);    
  }

  int get_size(){
    return size_l;
  }

  Node* get_head(){
    return head;
  }

};

int variation(float *l_g, LinkedList* list){
  float accumulator = 0;
  Node* current_node = list->get_head();
  int i = 0;
  while(current_node){
    accumulator +=l_g[i]*current_node->value;
    i++;
    current_node = current_node->next;
  }
  return int(accumulator);
}

