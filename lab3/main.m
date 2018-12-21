
%% im_align1
clear
close all
% check if the directory used for storing image is exist, if not, make such
% a directory
dirstring='imagefile';
checkdir=dir(dirstring);
if isempty(checkdir)
    mkdir(dirstring)
end
run('im_align1.m')
%% im_align2
clear
close all
dirstring='imagefile';
run('im_align2.m')
%% im_align3
clear
close all
dirstring='imagefile';
run('im_align3.m')
