numberOfTags = 0
order = ""
orderList = []
def GetInput():
    global numberOfTags
    global order
    global orderList
    print("Put in the number of tags")
    numberOfTags = input()

    print("Put in the order the tags were found")
    order = input()
    orderList = split(order)

    if len(orderList) != int(numberOfTags):
        print("wrong Input try again")
        GetInput()

def split(word):
    return [int(char) for char in word]

def main():
    global orderList
    GetInput()
            
    print("The following tags have been found: ")
    for i in range(len(orderList)):
        print("Tag number "+ str(i) + " has the global number " + str(orderList[i]))

    f = open("globalTagNumbers.txt","w")
    f.write(str(orderList))

    return orderList

if __name__ == '__main__':
    main()
