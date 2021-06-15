from django.http import HttpResponse
from django.shortcuts import render

# views - handles the various webpages

def home_view(request, *args, **kwargs):
    return render(request, "pages/home_page.html", {})