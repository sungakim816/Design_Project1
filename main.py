from Tools.solar import Solar

def main():
    test = Solar()
    try:
        while True:
            test.get_image()
            if test.is_there_sun:
                test.mark_sun()
                test.print_sun_coordinates() 
            test.show_image()                          
    except KeyboardInterrupt:
            print('Terminated') 

# main thread
main()
