from django.shortcuts import render

def map_route_view(request):
    context = {}
    
    return render(request, "map_route/grid_map.html", context)


