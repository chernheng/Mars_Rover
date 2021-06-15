# Web App Mars Rover
## Package Dependencies
In a new environment, make sure that python3 is installed by running the command `python3 --version`
 - if not, install python3 by executing `sudo apt install python3.6`

Make sure pip3 is installed by running the command `pip3 --version`
- if not, install pip3 by executing `sudo apt install python3-pip`

Creating a new virtual environment:
- `python -m pip install pipenv`
- `pipenv install`

Navigate to the target directory and run `pipenv shell`

Set up the virtual environment by running the command
`pip install -r requirements.txt`
This will download all the packages required for the web app.

Follow the instructions in https://redis.io/download to download Redis.

## Django Web App
Setting up the app for the first time:
1) Database migrations
- `cd ./trydjango/src`
- run `python manage.py makemigrations`
- run `python manage.py migrate`
2) Setting up admin account
To set up the admin account for the first time:
- run `python manage.py createsuperuser` and follow the command prompts

To get the Web App up and running:
1) Run the main web server:
- Edit `./trydjango/src/trydjango/settings.py` line 28 to `ALLOWED_HOSTS = [<public ip address of cloud server>, "127.0.0.1"]`
- cd `./trydjango/src/`
- run `python manage.py runserver <internal ip address of cloud server>:8000`
2) Run the Redis backend datastore and message broker:
- `redis-server`
3) Run the celery worker process:
- `celery -A trydjango worker -l info`

## Accessing the User Interface
Access the user web display by entering `http://<external ip address of cloud server>:8000` 
