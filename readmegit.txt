¡¦or create a new repository on the command line
echo "# nanobio" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/samdori93/nanobio.git
git push -u origin main

¡¦or push an existing repository from the command line
git remote add origin https://github.com/samdori93/nanobio.git
git branch -M main
git push -u origin main

sudo git add .
sudo git commit -m '2021-07-17'
sudo git push