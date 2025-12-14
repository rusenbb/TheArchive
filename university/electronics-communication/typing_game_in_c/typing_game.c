#include <Windows.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#define ESC 27     // Defined escape key
#define PLANE_Y 27 // y cord of plane
#define buildingHeight 6
#define MAX_NUM_OF_WORDS 10
#define DIFFICULTY_SCALE 1.8

//GLOBAL VARIABLES
int score = 0;
int difficulty = 3;        // difficulty = word length (Max 10- Min 3)
int difficultyCounter = 0; // counter to checks for 5 correct words written
int planex = 1;            // x cord of plane
struct _bomb
{
    int ycord;
    int xcord;
} bomb = {PLANE_Y - 3, 1};

char word[11]; // for 10 letter words 11th character is needed for
               // \0 (termination) character

// LITTLE HELPER FUNCTIONS
int randomWord(int length)
{
    // Works on codewords.txt and takes the words that are
    // specified 'int length' parameter long. Adds them into
    // an array, picks one word and equilize 'global word' to that word
    // in short: Changes 'global word' to a random specified length word
    FILE *fp = fopen("codewords.txt", "r");
    char c[11];
    char wordPool[MAX_NUM_OF_WORDS][11];
    int wordcount = 0;

    while ((fscanf(fp, "%s", c)) != EOF)
    {
        if (strlen(c) != length)
        {
            continue;
        }

        for (int i = 0; i < 11; i++)
        {
            wordPool[wordcount][i] = c[i];
        }
        wordcount++;
    }
    fclose(fp);
    srand(time(NULL));
    int randnum;
    randnum = rand() % (wordcount);

    for (int i = 0; i < 11; i++)
    {
        word[i] = wordPool[randnum][i];
    }
}

int shiftWordLeft(char array[])
{
    // Shifts a character array to the left (errases left most character)
    for (int i = 0; i < 11; i++)
    {
        array[i] = array[i + 1];
    }
}

int printSpaceFor(int xcoord)
{
    // Prints space for specified amount
    for (int i = 0; i < xcoord; i++)
    {
        printf(" ");
    }
}

int newgame()
{
    // reinitilazes global parameters (except planex (not needed))
    difficulty = 3;
    planex = 1;
    score = 0;
    bomb.xcord = planex;
    bomb.ycord = PLANE_Y - 3;
    randomWord(difficulty); // word will be 3 letter random word
    return 0;
}

int save()
{ //Saves the current game to a file
    printf("Enter the name of the file (e.g. file.dat, ./folder/file.txt): ");
    char name[200];
    scanf("%s", name);
    FILE *fp;
    fp = fopen(name, "w+");
    fprintf(fp, "difficulty: %d, difficultyCounter: %d, planex: %d, score: %d, bombx: %d, bomby: %d, word: %s",
            difficulty, difficultyCounter, planex, score, bomb.xcord, bomb.ycord, word);
    printf("Game is succesfully saved ^_^\n");
    printf("Returning to the game in 3 seconds.\n");
    sleep(3);
    fclose(fp);
}

int loadgame()
{
    // Loads a saved game
    FILE *fp;
    long numbytes;
    printf("Enter the name of the file (e.g. file.dat, ./folder/file.txt): ");
    char name[200];
    scanf("%s", name);
    /* open an existing file for reading */
    fp = fopen(name, "r");

    /* quit if the file does not exist */
    if (fp == NULL)
    {
        printf("FILE DOESN'T EXIST\n");
        return 1;
    }

    fscanf(fp, "difficulty: %d, difficultyCounter: %d, planex: %d, score: %d, bombx: %d, bomby: %d, word: %s",
           &difficulty, &difficultyCounter, &planex, &score, &(bomb.xcord), &(bomb.ycord), word);

    printf("Loading the game in 3 seconds.\n");
    sleep(3);
    return 0;
}

// MAIN FUNCTIONS THAT DOES THE JOB
int menu()
{
    // Pints the menu.txt context and expects for a valid button
    // input. Then switches to the specified option (function)
    int button = -1;
    FILE *fp = fopen("menu.txt", "r");
    char c;
    while (!((button > 0) && (button < 6)))
    {
        system("cls");
        while ((c = fgetc(fp)) != EOF)
        {
            putchar(c);
        }
        printf("\n");
        scanf("%d", &button);
    }
    int isError = 0;
    switch (button)
    {
    case 1: // New game
        newgame();
        break;
    case 2: // Load a saved game
        isError = loadgame(difficulty, planex, score);
        if (!isError)
            break;
        printf("NO SUCH FILE. Returning home in 3 seconds.");
        sleep(3);
        menu();
        break;
    case 3: // Save current game
        save(difficulty, planex, score);
        break;
    case 4: // Return to game
        printf("Returning to the game in 3 seconds.\n");
        sleep(3);
        break;
    case 5: // Exit
        exit(0);
        break;
    default:
        break;
    }

    return 0;
}

int displayMainGame()
{
    // Function to print basically everything
    // prints the maing game to the screen
    system("cls");

    FILE *fp;
    char *buffer;
    char c;

    //printing the score
    printf("%68s%6d\n", "Score: ", score);

    //printing the plane
    fp = fopen("plane.txt", "r");

    printSpaceFor(planex);
    while ((c = getc(fp)) != EOF)
    {
        putchar(c);
        if (c == '\n')
        {
            printSpaceFor(planex);
        }
    }
    printf("\n");
    fclose(fp);

    // empty lines between the plane and the bomb
    for (int i = 0; i < PLANE_Y - bomb.ycord - 3; i++)
    {
        printf("\n");
    }

    //printing the bomb
    fp = fopen("bomb.txt", "r");

    printSpaceFor(bomb.xcord);
    while ((c = getc(fp)) != EOF)
    {
        if (c == '\n')
        {
            putchar('\n');
            printSpaceFor(bomb.xcord);
        }
        else
            putchar(c);
    }

    //printing the word
    printf("%s", word);

    printf("\n");
    fclose(fp);

    // empty lines between the bomb and the city
    for (int i = 0; i < bomb.ycord - buildingHeight - 3; i++)
    {
        printf("\n");
    }

    //printing the city
    fp = fopen("city.txt", "r");
    while ((c = getc(fp)) != EOF)
    {
        putchar(c);
    }
    fclose(fp);
}

// MASTER MAIN FUNCTION
int main()
{
    clock_t clock_time1, clock_time2;
    clock_time1 = clock();

    randomWord(difficulty); // initilaze the global word before the infinite main loop
    while (1)               // infinite loop
    {
        while (_kbhit()) // while user presses keyboard
        {
            char c = getch();
            if (c == ESC) // check for ESC press
            {
                menu();
            }
            else if (c == word[0]) // check if the pressed key is correct
            {
                shiftWordLeft(word); // delete the correct character entered
                score++;
            }
            else // else penalty
            {
                score--;
            }
            displayMainGame(); // refresh screen again to let user see if he/she/
        }

        //check if reached end of the word
        if (word[0] == '\0')
        {
            if ((difficultyCounter % 5 == 0) && (difficulty < 10) && (difficultyCounter != 0))
            {
                difficultyCounter = 0; // set difficultyCounter to 0 for other 5 strikes
                difficulty++;
            }
            randomWord(difficulty);
            bomb.ycord = PLANE_Y;
            bomb.xcord = planex;
            difficultyCounter++;
        }
        clock_time2 = clock();

        // This is the clock check, loop can run on different speeds on different machines
        // but (clock_time2 - clock_time1) / CLOCKS_PER_SEC) will always be the seconds passed since
        // last clock check. And program will always refresh screen based on the difficulty scale and difficulty
        if (((float)(clock_time2 - clock_time1) / CLOCKS_PER_SEC) > DIFFICULTY_SCALE / difficulty)
        {
            bomb.ycord--;
            planex++;
            if (planex == 35)
                planex = 1;
            clock_time1 = clock_time2;
            displayMainGame();
        }

        if (bomb.ycord <= buildingHeight + 3) // bomb hit the city: GAME OVER
        {
            char ask;
            system("cls");
            printf("********************************\n");
            printf("********************************\n");
            printf("**********GAME OVER*************\n");
            printf("********************************\n");
            printf("********************************\n");
            printf("Your score is: %d\n", score);
            printf("Would you like to start a new game (y/n): \n");
            scanf("%c", &ask);
            if (ask == 'y')
                newgame();
            else
                exit(0);
        }
    }
}