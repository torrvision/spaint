% rootFolders = dir;
% sequences = {rootFolders([rootFolders.isdir]).name};
% datasets = datasets(~ismember(datasets,{'.','..'}));
%datasets = datasets(~ismember(datasets,{'chess','office'})); % just to avoid repeating work
%datasets = {'stairs'};

sequences = {'chess', 'fire', 'heads', 'office', 'pumpkin', 'redkitchen', 'stairs'};
sequence_sizes = [1000 1000 1000 1000 1000 1000 500];

calibrationFile = 'calib.txt';

tic;
for sequence = sequences
    sequenceName = sequence{1};
    fprintf('Processing sequence %s...\n', sequenceName);
    
    % Process Training dataset
    trainingInFolder = fullfile(sequenceName, 'TrainSequences');
    fprintf('Training folder: %s\n', trainingInFolder);
    
    outputDir = fullfile(sequenceName, 'train');
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end
    copyfile(calibrationFile, outputDir);
    
    outCount = 0;
    trainingSeqs = dir(fullfile(trainingInFolder, 'seq-*'));
    for seqId = 1:length(trainingSeqs)
        seqName = trainingSeqs(seqId).name;
        fprintf('\tProcessing sequence: %s\n', seqName);
        seqFolder = fullfile(trainingInFolder, seqName);
        
%         for i = 0:999
       for i = 0:499 % stairs
            rgbInName = sprintf('%s/frame-%06i.color.png', seqFolder, i);
            depthInName = sprintf('%s/frame-%06i.depth.png', seqFolder, i);
            poseInName = sprintf('%s/frame-%06i.pose.txt', seqFolder, i);

%             rgbOutName = sprintf('%s/rgbm%06i.ppm', outputDir, outCount);
%             depthOutName = sprintf('%s/depthm%06i.pgm', outputDir, outCount);
%             poseOutName = sprintf('%s/posem%06i.txt', outputDir, outCount);
            rgbOutName = sprintf('%s/frame-%06i.color.png', outputDir, outCount);
            depthOutName = sprintf('%s/frame-%06i.depth.png', outputDir, outCount);
            poseOutName = sprintf('%s/frame-%06i.pose.txt', outputDir, outCount);
            
%             imwrite(imread(rgbInName), rgbOutName);
%             imwrite(imread(depthInName), depthOutName);

            depthIm = imread(depthInName);
            depthIm(depthIm==65535) = 0;
            imwrite(depthIm, depthOutName);

            copyfile(poseInName, poseOutName);
%             copyfile(depthInName,depthOutName);
            copyfile(rgbInName, rgbOutName);
            
            outCount = outCount + 1;
        end
    end
    
    % Process Testing dataset
    testingFolder = fullfile(sequenceName, 'Test');
    fprintf('Testing folder: %s\n', testingFolder);
    
    outputDir = fullfile(sequenceName, 'test');
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end
    copyfile(calibrationFile, outputDir);
    
    outCount = 0;
    testingSeqs = dir(fullfile(testingFolder, 'seq-*'));
    for seqId = 1:length(testingSeqs)
        seqName = testingSeqs(seqId).name;
        fprintf('\tProcessing sequence: %s\n', seqName);
        seqFolder = fullfile(testingFolder, seqName);
        
%         for i = 0:999
        for i = 0:499 % stairs
            rgbInName = sprintf('%s/frame-%06i.color.png', seqFolder, i);
            depthInName = sprintf('%s/frame-%06i.depth.png', seqFolder, i);
            poseInName = sprintf('%s/frame-%06i.pose.txt', seqFolder, i);

%             rgbOutName = sprintf('%s/rgbm%06i.ppm', outputDir, outCount);
%             depthOutName = sprintf('%s/depthm%06i.pgm', outputDir, outCount);
%             poseOutName = sprintf('%s/posem%06i.txt', outputDir, outCount);
            rgbOutName = sprintf('%s/frame-%06i.color.png', outputDir, outCount);
            depthOutName = sprintf('%s/frame-%06i.depth.png', outputDir, outCount);
            poseOutName = sprintf('%s/frame-%06i.pose.txt', outputDir, outCount);
            
%             imwrite(imread(rgbInName), rgbOutName);
%             imwrite(imread(depthInName), depthOutName);

            depthIm = imread(depthInName);
            depthIm(depthIm==65535) = 0;
            imwrite(depthIm, depthOutName);

            copyfile(poseInName, poseOutName);
%             copyfile(depthInName,depthOutName);
            copyfile(rgbInName, rgbOutName);
            
            outCount = outCount + 1;
        end
    end
end
toc
