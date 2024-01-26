`#include <vector>`

### init 

```cpp
std::vector<int> numbers;
std::vector<double> prices(3, 5); // siez, init value
std::vector<std::string> names = {"alice", "bob", "charlie"};
```

### method 

```cpp
numbers.push_back(42);
numbers.pop_back(); // delete 42

// delete
numbers.clear();    // clear all 
numbers.erase(numbers.begin() + 2, numbers.begin()+3); // delete at(2) and at(3)

// insert 
numbers.insert(numbers.begin()+2, 50); // insert 50 at(3)

// visit
numbers[0];
number.at(0);

// size
numbers.size();

// traverse 
for (const auto& num: numbers){
	std::cout << num << " ";
}

// check empty
bool is_empty = numbers.empty();
```


