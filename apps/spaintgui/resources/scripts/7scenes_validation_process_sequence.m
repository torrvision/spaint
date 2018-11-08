function process_sequence(inFolder, outFolder, sequenceLength)

    if ~exist(outFolder, 'dir')
        mkdir(outFolder);
    end

    outCount = 0;
    subSeqs = dir(fullfile(inFolder, 'seq-*'));
    
    for seqId = 1:length(subSeqs)
        
        seqName = subSeqs(seqId).name;
        fprintf('\tProcessing sequence: %s\n', seqName);
        seqFolder = fullfile(inFolder, seqName);
        
       for i = 0:(sequenceLength - 1)
            rgbInName = sprintf('%s/frame-%06i.color.png', seqFolder, i);
            depthInName = sprintf('%s/frame-%06i.depth.png', seqFolder, i);
            poseInName = sprintf('%s/frame-%06i.pose.txt', seqFolder, i);

            rgbOutName = sprintf('%s/frame-%06i.color.png', outFolder, outCount);
            depthOutName = sprintf('%s/frame-%06i.depth.png', outFolder, outCount);
            poseOutName = sprintf('%s/frame-%06i.pose.txt', outFolder, outCount);

            copyfile(poseInName, poseOutName);
            
            depthIm = imread(depthInName);
            depthIm(depthIm==65535) = 0;
            imwrite(depthIm, depthOutName);

            copyfile(rgbInName, rgbOutName);
            
            outCount = outCount + 1;
        end
    end

end