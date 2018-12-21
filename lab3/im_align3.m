
separcell={[1,348;349,689;690,1024],[1,348;349,685;686,1024],[1,340;341,680;681,1024],...
    [1,338;339,675;676,1024],[1,344;345,685;686,1024],[1,335;336,674;675,1024]};% lines at which multipe imgages shoudl be separated

kapa=0.1;
step=1; %pixcel step to calculate cornerness
topnum=200;% feature number to be extracted
numransac=180;
win=5;% window size to cacluate cornerness between two patches of image
th_supp=9;% window size to calculate local maxima when revisting cornerness matrix
threshold=3;% window size to detect if find corresponding feature in second image
th_cor=0; % only retain the corner feature above this threshold
for ii=1:6
    clear Rim Gim Bim Rim_shift Gim_shift imcolor
    num=ii;
    sprintf("For image %d",num)
    
    imagename=strcat("image",num2str(num),".jpg");
    image_orig=imread(imagename);
    col=length(image_orig(1,:));
    separator=separcell{num};
    Bsize=separator(1,2)-separator(1,1);% top
    Gsize=separator(2,2)-separator(2,1);% middile
    Rsize=separator(3,2)-separator(3,1);% bottom
    
    sizediff=abs([Bsize,Gsize,Rsize]-max([Bsize,Gsize,Rsize]));
    Bim=[image_orig(separator(1,1):separator(1,2),:);ones(sizediff(1),col)*255];% top B
    Gim=[image_orig(separator(2,1):separator(2,2),:);ones(sizediff(2),col)*255];% middle G
    Rim=[image_orig(separator(3,1):separator(3,2),:);ones(sizediff(3),col)*255];% bottom R
    % Trim the image boundary to avoid too many corner detection on
    % boundaries
    [row,col]=size(Bim);
    choprow=floor(row*0.08);
    chopcol=floor(col*0.08);
    Bim=Bim(1+choprow:end-choprow,1+chopcol:end-chopcol);
    Gim=Gim(1+choprow:end-choprow,1+chopcol:end-chopcol);
    Rim=Rim(1+choprow:end-choprow,1+chopcol:end-chopcol);
    % corner detection
%    % Use Matlab build-in harris corner detector
%     C=corner(Bim,200);
%     Bim_posx=C(:,2);
%     Bim_posy=C(:,1);
%     Bim_fmatrix=zeros(size(Bim));
%     Bim_fmatrix(C(:,2),C(:,1))=1;
%     C=corner(Gim,200);
%     Gim_posx=C(:,2);
%     Gim_posy=C(:,1);
%     Gim_fmatrix=zeros(size(Gim));
%     Gim_fmatrix(C(:,2),C(:,1))=1;
%     C=corner(Rim,200);
%     Rim_posx=C(:,2);
%     Rim_posy=C(:,1);
%     Rim_fmatrix=zeros(size(Rim));
%     Rim_fmatrix(C(:,2),C(:,1))=1;
    
    [Bim_cornerness,Bim_fmatrix,Bim_posx,Bim_posy]=harris(Bim,win,kapa,topnum,step,th_supp,th_cor);
    [Gim_cornerness,Gim_fmatrix,Gim_posx,Gim_posy]=harris(Gim,win,kapa,topnum,step,th_supp,th_cor);
    [Rim_cornerness,Rim_fmatrix,Rim_posx,Rim_posy]=harris(Rim,win,kapa,topnum,step,th_supp,th_cor);

    figure((ii-1)*3+1)
    subplot(2,2,1)
    imshow(Bim)
    hold on
    plot(Bim_posy,Bim_posx,'o','MarkerSize',5,'color','r')
    subplot(2,2,2)
    imshow(Gim)
    hold on
    plot(Gim_posy,Gim_posx,'o','MarkerSize',5,'color','r')
    subplot(2,2,3)
    imshow(Rim)
    hold on
    plot(Rim_posy,Rim_posx,'o','MarkerSize',5,'color','r')
    saveas(gcf,strcat(dirstring,"/image",num2str(num),'-corner',".jpg"))

    [bestshift,maxinliner,feature_e]=ransac(Gim_fmatrix,Gim_posx,Gim_posy,Bim_fmatrix,Bim_posx,Bim_posy,threshold,numransac);
    sprintf("The max inliner for G channel is %d, the effective feature is %d, best shift is [%s]",...
        maxinliner,feature_e,num2str(bestshift))
    Gim_shift=circshift(Gim,bestshift);
    [bestshift,maxinliner,feature_e]=ransac(Rim_fmatrix,Rim_posx,Rim_posy,Bim_fmatrix,Bim_posx,Bim_posy,threshold,numransac);
    Rim_shift=circshift(Rim,bestshift);
    sprintf("The max inliner for R channel is %d, the effective feature is %d, best shift is [%s]",...
        maxinliner,feature_e,num2str(bestshift))

%     figure(3)
%     imshow(tmpl)
%     imwrite(tmpl,strcat("imagealign/template_",num2str(num),".jpg"))
    imcolor(:,:,1)=Rim_shift;
    imcolor(:,:,2)=Gim_shift;
    imcolor(:,:,3)=Bim;
    figure((ii-1)*3+2)
    imshow(imcolor)
    imwrite(imcolor,strcat(dirstring,"/image",num2str(num),'-ransac',".jpg")) % aligned image
%     
%     imcolor(:,:,1)=Rim;
%     imcolor(:,:,2)=Gim;
%     imcolor(:,:,3)=Bim; 
%     figure(1)
%     image(imcolor)
%     imwrite(imcolor,strcat("imagealign/Notaligned_",num2str(num),".jpg"))

end
% figure(2)
% figure(3)
function [bestshift,maxinliner,feature_effective]=ransac(im1_fmatrix,im1_posx,im1_posy,im2_fmatrix,im2_posx,im2_posy,threshold,numransac)
% This function calculates the best shift from feature matrix of image one
% to that of image two (im1 + diff= im2)
addrow=(threshold-1)/2;
if length(im1_posx)~=length(im2_posx)
    sprintf("Check the dimensions of two imput feature matrice")
else
    bestshift=zeros(1,2);
    feature_effective=0;
    maxinliner=-1;
    for kk=1:numransac % the number of ransac
        len_feature=length(im1_posx);
        [row,col]=size(im1_fmatrix);
        
        randidx=randi(len_feature);
        point1=[im1_posx(randidx),im1_posy(randidx)];% randomly pick a feature point in the first image
        randidx=randi(len_feature);
        point2=[im2_posx(randidx),im2_posy(randidx)];% randomly pick feature point in the second image
        diff=point2-point1;%
        
%         % shift the row coordinates in the first image(simulate circshift of feature matrix)
%         im1_posx_shift=im1_posx+diff(1);
%         index=im1_posx_shift>row;
%         im1_posx_shift(index)=im1_posx_shift(index)-row;
%         index=im1_posx_shift<1;
%         im1_posx_shift(index)=row+im1_posx_shift(index);% row + (x-1) +1
%         % shift the col coordinates in the first image
%         im1_posy_shift=im1_posy+diff(2);
%         index=im1_posy_shift>col;
%         im1_posy_shift(index)=im1_posy_shift(index)-col;
%         index=im1_posy_shift<1;
%         im1_posy_shift(index)=col+im1_posy_shift(index);
        
        % drop the features that are out of boundary
        im1_posx_shift=im1_posx+diff(1);
        im1_posy_shift=im1_posy+diff(2);
        index=im1_posx_shift>row|im1_posx_shift<1|im1_posy_shift>col|im1_posy_shift<1;
        im1_posx_shift(index)=[];
        im1_posy_shift(index)=[];

        im2_fmatrix_enlarge=zeros(row+2*addrow,col+2*addrow);% enlarge feature matrix by adding all-zero rows and colums(not feature)
        im2_fmatrix_enlarge(addrow+1:row+addrow,addrow+1:col+addrow)=im2_fmatrix;
        inliner=0;
        im1_posx_temp=im1_posx_shift+addrow;% adapt to enlarged im2_fmatrix
        im1_posy_temp=im1_posy_shift+addrow;
        for ii=1:length(im1_posx_shift) % loop the shifted feature coordinates defined in (orignal)smaller image
            window=im2_fmatrix_enlarge(im1_posx_temp(ii)-addrow:im1_posx_temp(ii)+addrow,...
                im1_posy_temp(ii)-addrow:im1_posy_temp(ii)+addrow);%must search in enlarged image find windows in second image
            if isempty(find(window==1, 1))
            else
                inliner=inliner+1;
            end
        end
        if maxinliner<inliner
            maxinliner=inliner;% update the number of maximum inliners
            bestshift=diff;% record best shift to algin the two image
            feature_effective=length(im1_posx_shift);
        else
        end
    end
end
end

