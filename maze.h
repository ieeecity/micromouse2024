struct Cell {
  bool walls[4]; // Array to represent walls: 0 -> right, 1 -> left, 2 -> up, 3 -> down
  int x; // X coordinate of the cell
  int y; // Y coordinate of the cell
  int weight; // weight of cell
};
Cell maze[8][8] = {
  {
    // R     L      U     D
    {{true, true, false, true}, 0, 0, 0}, 
    {{false, false, false, true}, 1, 0, 0}, 
    {{false, false, false, true}, 2, 0, 0}, 
    {{false, false, false, true}, 3, 0, 0}, 
    {{false, false, false, true}, 4, 0, 0}, 
    {{false, false, false, true}, 5, 0, 0}, 
    {{false, false, false, true}, 6, 0, 0}, 
    {{true, false, false, true}, 7, 0, 0}
  },

  {
    {{false, true, false, false}, 0, 1, 0}, 
    {{false, false, false, false}, 1, 1, 0}, 
    {{false, false, false, false}, 2, 1, 0}, 
    {{false, false, false, false}, 3, 1, 0}, 
    {{false, false, false, false}, 4, 1, 0}, 
    {{false, false, false, false}, 5, 1, 0}, 
    {{false, false, false, false}, 6, 1, 0}, 
    {{true, false, false, false}, 7, 1, 0}
  },

  {
    {{false, true, false, false}, 0, 2, 0}, 
    {{false, false, false, false}, 1, 2, 0}, 
    {{false, false, false, false}, 2, 2, 0}, 
    {{false, false, false, false}, 3, 2, 0}, 
    {{false, false, false, false}, 4, 2, 0}, 
    {{false, false, false, false}, 5, 2, 0}, 
    {{false, false, false, false}, 6, 2, 0}, 
    {{true, false, false, false}, 7, 2, 0}
  },

  {
    {{false, true, false, false}, 0, 3, 0}, 
    {{false, false, false, false}, 1, 3, 0}, 
    {{false, false, false, false}, 2, 3, 0}, 
    {{false, false, false, false}, 3, 3, 0}, 
    {{false, false, false, false}, 4, 3, 0}, 
    {{false, false, false, false}, 5, 3, 0}, 
    {{false, false, false, false}, 6, 3, 0}, 
    {{true, false, false, false}, 7, 3, 0}
  },

  {
    {{false, true, false, false}, 0, 4, 0}, 
    {{false, false, false, false}, 1, 4, 0}, 
    {{false, false, false, false}, 2, 4, 0}, 
    {{false, false, false, false}, 3, 4, 0}, 
    {{false, false, false, false}, 4, 4, 0}, 
    {{false, false, false, false}, 5, 4, 0}, 
    {{false, false, false, false}, 6, 4, 0}, 
    {{true, false, false, false}, 7, 4, 0}
  },

  {
    {{false, true, false, false}, 0, 5, 0}, 
    {{false, false, false, false}, 1, 5, 0}, 
    {{false, false, false, false}, 2, 5, 0}, 
    {{false, false, false, false}, 3, 5, 0}, 
    {{false, false, false, false}, 4, 5, 0}, 
    {{false, false, false, false}, 5, 5, 0}, 
    {{false, false, false, false}, 6, 5, 0}, 
    {{true, false, false, false}, 7, 5, 0}
  },

  {
    {{false, true, false, false}, 0, 6, 0}, 
    {{false, false, false, false}, 1, 6, 0}, 
    {{false, false, false, false}, 2, 6, 0}, 
    {{false, false, false, false}, 3, 6, 0}, 
    {{false, false, false, false}, 4, 6, 0}, 
    {{false, false, false, false}, 5, 6, 0}, 
    {{false, false, false, false}, 6, 6, 0}, 
    {{true, false, false, false}, 7, 6, 0}
  },

  {
    {{false, true, true, false}, 0, 7, 0}, 
    {{false, false, true, false}, 1, 7, 0}, 
    {{false, false, true, false}, 2, 7, 0}, 
    {{false, false, true, false}, 3, 7, 0}, 
    {{false, false, true, false}, 4, 7, 0}, 
    {{false, false, true, false}, 5, 7, 0}, 
    {{false, false, true, false}, 6, 7, 0}, 
    {{true, false, true, false}, 7, 7, 0}
  }
};

void printMaze(const struct Cell (&maze)[8][8]) {
  for(int y = 7; y >= 0; --y){ // For each ROW
    Serial.println("");
    for(int x = 0; x < 8; ++x){ // for each col value in said row

      if(maze[y][x].walls[1]){ // Left wall
        Serial.print("|");
      }

      Serial.print(maze[y][x].x);

      if(maze[y][x].walls[2]){ // Top wall
        Serial.print("¯");
        } else{
          Serial.print(",");
        }

      if(maze[y][x].walls[3]){ // Bottom wall
        Serial.print("_");
        } else{
          Serial.print(",");
        }
      Serial.print(maze[y][x].y);

      if(maze[y][x].walls[0]){ // Right wall
        Serial.print("|");
      }

      if(!(maze[y][x].x >= 10 || maze[y][x].y >= 10)){
        Serial.print(" ");
      } else{
        Serial.print("  ");
      }
    }
  }
  Serial.println(); // Print a newline after printing the entire maze
}

void printMazeWeights(const struct Cell (&maze)[8][8]) {
  for(int y = 7; y >= 0; --y){ // For each ROW
    Serial.print(y);
    Serial.print("  ");
    for(int x = 0; x < 8; ++x){ // for each col value in said row

      if(maze[y][x].walls[1]){ // Left wall
        Serial.print("|");
      }
      
      if(maze[y][x].weight < 0){
        Serial.print("?");
      } else{
        Serial.print(maze[y][x].weight);
      }
    
      if(maze[y][x].walls[2]){ // Top wall
        Serial.print("¯");
        } else{
          Serial.print(" ");
        }

      if(maze[y][x].walls[3]){ // Bottom wall
        Serial.print("_");
        } else{
          Serial.print(" ");
        }
        
      if(maze[y][x].walls[0]){ // Right wall
        Serial.print("|");
      }

      if(!(maze[y][x].walls[0]) && maze[y][x].weight <10){
	Serial.print("   ");
      } else if(!(maze[y][x].walls[0])){
	  Serial.print("  ");
	}
    }
    Serial.println(""); // each row on a new line
  }
  Serial.println("");
  Serial.println("      0     1     2     3     4     5     6     7   ");
}
