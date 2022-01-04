 #!/usr/bin/python3

maximum_number = 10000

def isPrime(value):
    print('Vendo se '+ str(value)+ ' é primo: ')
    for i in range(2,value):
        resto=value % i
        print('Divisão por '+str(value)+ ' is ' + str(resto))
        if resto==0:
            print('O numero '+str(value)+' não é primo porque a divisão por '+str(i)+' é 0')
            return False
    return True
def main():
    print("Starting to compute prime numbers up to " + str(maximum_number-1))
    contador=0
    for i in range(1, maximum_number):
        if isPrime(i):
            print('Number ' + str(i) + ' is prime.')
            contador+=1
        else:
            print('Number ' + str(i) + ' is not prime.')
    print('Entre 1 e ' + str(maximum_number)+' ha '+str(contador)+'numeros primos')
if __name__ == "__main__":
    main()
