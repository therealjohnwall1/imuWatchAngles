#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#define LINE_BUF_SIZE 20
#define K_NEIGH 3
#define FILE_NAME "values.csv"

struct dataPoint {
    float euclidDist;
    int class;
};

float euclidDistance(float x1, float y1, float z1, float x2, float y2, float z2){
    return sqrt(pow(x2-x1,2) + pow(y2-y1,2) + pow(z2-z1,2));
}

float* parseStringtoArr(const char* str, int num_floats) {
    float* res = (float*) malloc(num_floats * sizeof(float));
    char temBuffer[10];
    int tempDex = 0;
    int col= 0;
    int i = 0;
    while(str[i] != '\0') {
        if (str[i] != ',') {
            temBuffer[tempDex++] = str[i];
        }
        else {
            temBuffer[tempDex] = '\0';
            res[col] = strtof(temBuffer, NULL);
            tempDex = 0;
            col++;
        }
        i++;
    }
    assert(tempDex > 0);
    temBuffer[tempDex] = '\0';
    res[col] = strtof(temBuffer, NULL);
    // assert(col == num_floats);
    return res;
}


float** fillTable () {
    int rows = 101 ;
    int cols = 4;
    FILE* values = fopen(FILE_NAME, "r");
    float** table = (float**)malloc(rows * sizeof(float*));

    char buffer[LINE_BUF_SIZE];
    int i = 0;
    fgets(buffer, LINE_BUF_SIZE,values);

    while(fgets(buffer, LINE_BUF_SIZE, values)) {
        table[i] = parseStringtoArr(buffer, cols);     
        i++;
    }

    for(int r = 0; r < i; r++) {
        for(int z = 0; z < 4; z++){
            // printf("%f, ", table[r][z]);
        }
        // printf("\n");

    }
    fclose(values);
    return table;
}

int compare(const void *euc1, const void *euc2) {
    return (((struct dataPoint*)euc1)->euclidDist - ((struct dataPoint*)euc2)->euclidDist);
}

int classify(int kVal, int* entryPoint, float** values, int valuesSize) { 
    struct dataPoint points[valuesSize];
    for(int i = 0; i < valuesSize;i++) {
        points[i].euclidDist = euclidDistance(entryPoint[0], entryPoint[1], entryPoint[2], values[i][0], values[i][1], values[i][2]);
        points[i].class = values[i][3];
        // printf("point: %d\n", points[i].class);
    }

    qsort(points, valuesSize, sizeof(struct dataPoint), compare);
    // printf("top k values: \n");

    int class0 = 0, class1= 0, class2= 0;
    for(int i = 0; i < valuesSize;i++) {
        int class = points[i].class;
        // printf("%d\n", class);
        switch(class) {
            case 0:
                class0++;
            case 1:
                class1++;
            case 2:
                class2++;
        }
        // printf("%d\n", points[i].class);
    }
    if (class0 >= class1 && class0 >= class2) {
        return 0;
    }
    else if (class1 >= class0 && class1 >= class2) {
        return 1;
    }
    else {
        return 2;
    }
}

int main() {
    float** vals = fillTable();
    int valuesSize = 100;
    int entry[3] = {5,2,3};
    int predClass = classify(K_NEIGH,entry, vals, valuesSize);
    printf("Predicted class: %d\n", predClass);
    // return 0;
}