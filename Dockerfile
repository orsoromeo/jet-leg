# syntax=docker/dockerfile:1

FROM python:3.8-slim-buster

WORKDIR /app

RUN apt-get update && apt-get install -y cython libglpk-dev 

COPY requirements.txt requirements.txt
COPY setup.py setup.py

RUN pip3 install -r requirements.txt

COPY /src src

CMD [ "python3", "-m" , "flask", "run", "--host=0.0.0.0"]
