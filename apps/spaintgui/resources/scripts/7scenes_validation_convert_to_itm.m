sequences = {'chess', 'fire', 'heads', 'office', 'pumpkin', 'redkitchen', 'stairs'};
sequence_sizes = [1000 1000 1000 1000 1000 1000 500];

calibrationFile = 'calib.txt';

tic;
for sequenceId = 1:length(sequences)
    sequenceName = sequences{sequenceId};
    fprintf('Processing sequence %s...\n', sequenceName);
    
    % Process Training dataset
    trainingFolder = fullfile(sequenceName, 'TrainSequences');
    fprintf('Training folder: %s\n', trainingFolder);   
    outputDir = fullfile(sequenceName, 'train');    
    process_sequence(trainingFolder, outputDir, sequence_sizes(sequenceId));
    copyfile(calibrationFile, outputDir);
    
    % Process Validation dataset
    validationFolder = fullfile(sequenceName, 'ValidationSequences');
    fprintf('Validation folder: %s\n', validationFolder);    
    outputDir = fullfile(sequenceName, 'validation');
    process_sequence(validationFolder, outputDir, sequence_sizes(sequenceId));
    copyfile(calibrationFile, outputDir);
    
    % Process Testing dataset
    testingFolder = fullfile(sequenceName, 'TestSequences');
    fprintf('Testing folder: %s\n', testingFolder);    
    outputDir = fullfile(sequenceName, 'test');
    process_sequence(testingFolder, outputDir, sequence_sizes(sequenceId));
    copyfile(calibrationFile, outputDir);
end
toc
