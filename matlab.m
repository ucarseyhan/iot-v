numberOfRandomNumber = 100;
maxValue = 100;
randomNumbers = randperm(numberOfRandomNumber,maxValue);
reshapedRandomNumbers = reshape(randomNumbers,[],5);
dlmwrite('output.txt',reshapedRandomNumbers,'delimiter','~')
