numberOfTokens = 0
order = ""
orderList = []

def GetInput():
    global numberOfTokens
    global order
    global orderList
    print("Put in the number of tags")
    numberOfTokens = input()
    return numberOfTokens

def split(word):
    return [int(char) for char in word]

def main():
    global orderList
    i = GetInput()
            
    print("There are " + str(i) + " tokens in the labyrinth.")

    return i

if __name__ == '__main__':
    main()
