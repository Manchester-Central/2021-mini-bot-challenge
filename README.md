# 2021 Mini Bot Challenge

This code is CHAOS' (FRC TEAM 131) code for controlling the [Romi minibot](https://docs.wpilib.org/en/latest/docs/romi-robot/index.html) for the [NE FIRST Mini Bot Challenge](https://nefirst.org/2021/01/new-england-launches-bae-systems-mini-bot-challenge/).

## Important Links
- [Chaos's Website](http://chaos131.com/)
- [Source Code](https://github.com/Manchester-Central/2021-mini-bot-challenge/tree/main/src/main/java/frc/robot)
- [Git Handbook](https://guides.github.com/introduction/git-handbook/)

## Setup Instructions

1. Install [2021 WPILib VS Code](https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/wpilib-setup.html) for Java development
2. [Configure the Romi](https://docs.wpilib.org/en/latest/docs/romi-robot/imaging-romi.html) for home use
3. Connect the Romi and Computer to the same network
    - Connect to the Raspberry Pi's network set up during step #2
    - Or configure the Raspberry Pi to connect to your own network (a guide is needed for that)
4. Download this project from Gitub and open it with 2021 WPILib VS Code and [run the program](https://docs.wpilib.org/en/latest/docs/romi-robot/programming-romi.html#running-a-romi-program)

### Getting the Romi's IP Address:
When connected to the same network as the Romi, you can run the following command to get the Romi's IP address:
``` batch
ping wpilibpi -4 -n 1
```

## Git helpers (for team members learning git)

These commands will help you do some of the basic git actions we will need to remotely work on this project

### Clone the project from Github

After having git installed (we use git built in to [Cmder](https://cmder.net/)), you can run the following command in the folder you want to copy the project to on your local computer:
```
git clone https://github.com/Manchester-Central/2021-mini-bot-challenge.git
```

### Check the status of your changes

Run the following command to see what changes you have made and what branch you are current on:
```
git status
```

### Process for uploading your local changes

We want to teach you how to do Pull Requests, so that you can upload your changes to Github, and let others review your work before we merge it into the `main` branch.

Before making any changes, make sure you are on the main branch and up to date:
```
git status
git checkout main
git pull origin main
```

If you have uncommited changes, `git status` will tell you. 
If you want to delete them, see [Dealing with unwanted changes](#dealing-with-unwanted-changes).
If you want to save them for later, see [Stashing or committing unready changes](#stashing-or-committing-unready-changes).

After that, create a new branch you want to work on:
```
git checkout -b your-branch-name
```
Where `your-branch-name` is whatever you want to call it, but make sure it has meaning, because others will see it later.

Make the changes you want to the project.

When your changes are ready to be commited and uploaded, run this:
```
git add -A
git commit -m "your commit message goes here"
git push origin your-branch-name
```

This will commit your changes and publish the branch to the [project's branch list](https://github.com/Manchester-Central/2021-mini-bot-challenge/branches), where you can create a Pull Request for others to review.

If you make more changes, you can run the git add, commit, and push commands again to update your branch on Github. If you have an Pull Request created, it will automatically update when you do this.

### Dealing with unwanted changes

If you have changes you want to get rid of and you want to go back to the `main` branch as it is on Github, you can run the following commands:
```
git fetch origin
git reset --hard origin/main 
```

### Stashing or committing unready changes

If you are not on the `main` branch, but you have changes you want to save before checking out `main`, you can commit your changes as you would normally.

If you are on `main`, but you don't want to lose your changes, you can stash your changes for later:
```
git stash
```

And use the following to unstash your changes (adding them back to the current branch):
```
git stash pop
```