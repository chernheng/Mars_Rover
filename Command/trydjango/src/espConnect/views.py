from django.shortcuts import render
from django.views.decorators.csrf import csrf_exempt
from csv import writer

@csrf_exempt
def update_server(request):
    return render(request, "espConnect/update.html", {})

def append_list_as_row(file_name, list_of_elem):
    # Open file in append mode
    with open(file_name, 'a+', newline='') as write_obj:
        # Create a writer object from csv module
        csv_writer = writer(write_obj)
        # Add contents of list as last row in the csv file
        csv_writer.writerow(list_of_elem)