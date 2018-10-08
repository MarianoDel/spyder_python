a = "Mariano"
b = "Chau"

print ("Hola " + a + ", " + b)
print (a)

#z = 1. / 3.
z = 1 / 3
print (z)
print ("el numero es: " + str(z))

print ("el maldito")

a_list = {1, "treu", 2, 5}

for i in a_list:
    print ("each element: " + format(i))

##Otra prueba
a_list = ['this', 'is', 'awesome!']

def print_a_list(a_list):
    for item in a_list:
        print(item)

print_a_list(a_list)

a_list.append('three_added')

print_a_list(a_list)

## string literals
# empiezo el string con f o F antes de la comilla simple
year = 2016
event = 'Referendum'
f'Results of the {year} {event}'

## str.format() sigo indicando donde quiero la variables con {}
yes_votes = 42572654
no_votes = 43132495
percentage = yes_votes / (yes_votes + no_votes)
'{:-9} YES votes  {:2.2%}'.format(yes_votes, percentage)
#>>> ' 42572654 YES votes  49.67%'
name = 'gil'
phone = '0800-gil'
print(f'{name:10} ==> {phone:10d}')

##formateos especiales
print ('el numero es {:03d} en 3 digitos'.format(95))
#el numero es 095 en 3 digitos
# {1:2d} {0:3d} segundo argumento en format 2 digitos, primer argumento 3 digitos
# https://docs.python.org/3/tutorial/inputoutput.html

# #print ("done")



