from numpy import number


numberOfTokens = 0
order = ""
orderList = []

class tokenOrder:
    def GetInput():
        global numberOfTokens
        global order
        global orderList

        if(int(numberOfTokens) == 0):
            print("Put in the number of tokens: ")
            numberOfTokens = input()
        else:
            print("There are %i tokens found" %int(numberOfTokens))
        print("Put in the order the tokens were found: ")
        order = input()
        orderList = tokenOrder.split(order)

        if len(orderList) != int(numberOfTokens):
            print("Wrong input. Try again!")
            tokenOrder.GetInput()

    def split(word):
        return [int(char) for char in word]

    def main(argNumberOfTokens):
        global numberOfTokens
        global orderList
        numberOfTokens = argNumberOfTokens
        tokenOrder.GetInput()
                
        print("The following tokens have been found: ")
        for i in range(len(orderList)):
            print("Token number "+ str(i) + " has the global number " + str(orderList[i]))

        f = open("globalTokenNumbers.txt","w")
        f.write(str(orderList))
        print("Token order is saved to file!")

if __name__ == '__main__':
    tokenOrder.main(5)
