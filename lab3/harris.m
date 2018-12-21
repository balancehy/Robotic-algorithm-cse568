% - dim is the window size to calculate ssd between two patches of image,
% could be 3-by-3, 5-by-5 etc.
% - step(>=1) is the pixcel step to calculate cornerness, the smaller
% the more coefficient will be calculated.
% - kapa is the coefficient in cornerness definition 0.04~0.15
% - topnum defines the number of feature that tops in the cornerness values
% - th_supp is window size to calculate local maxima 
function [cornerness,featurematrix,posx,posy]=harris(im,dim,kapa,topnum,step,th_supp,th_cor)
[row,col]=size(im);
addrow=(dim-1)/2;% add rows and columns to original image according to the window size
im_modi=zeros(row+2*addrow,col+2*addrow);
im_modi(addrow+1:row+addrow,addrow+1:col+addrow)=im;% create an bigger image including the original one
% the boundary of im_modi copies the boundary of im
if addrow==0 % if no row or column is added
else
   im_modi(:,1:addrow)=im_modi(:,addrow+1)*ones(1,addrow);
   im_modi(:,col+addrow+1:end)=im_modi(:,col+addrow)*ones(1,addrow);
   im_modi(1:addrow,:)=ones(addrow,1)*im_modi(addrow+1,:);
   im_modi(row+addrow+1:end,:)=ones(addrow,1)*im_modi(row+addrow,:);
end
% enlarge im_modi by adding one row and one column for gradient calculation
im_enlarge=[im_modi(:,1),im_modi,im_modi(:,end)];
im_enlarge=[im_enlarge(1,:);im_enlarge;im_enlarge(end,:)];
% calculate cornerness
cornerness=zeros(row,col);
for ii=addrow+1:step:row+addrow % loop the pixels in the inner image with defined step.
    for jj=addrow+1:step:col+addrow % ii, jj are defined in outer image coordinate
        M=[0,0;0,0];
        for i_w=ii-addrow:ii+addrow % i_w is defined in outer image
            for j_w=jj-addrow:jj+addrow
                [gx,gy]=getGradient(im_enlarge,i_w,j_w);% calculate gradient at each pixel in defined window region
                M=M+[gx;gy]*[gx,gy];
            end
        end
        cornerness(ii-addrow,jj-addrow)=det(M)-kapa*trace(M);%record cornerness 
    end
end
% Nonmaxima suppression to refine the cornerness matrix
addrow_supp=(th_supp-1)/2;
for ii=1:row
    for jj=1:col
        row_l=max(ii-addrow_supp,1);
        row_u=min(ii+addrow_supp,row);
        col_l=max(jj-addrow_supp,1);
        col_u=min(jj+addrow_supp,col);
        patch=cornerness(row_l:row_u,col_l:col_u);
        localmaxima=max(max(patch));
        if cornerness(ii,jj)==localmaxima && localmaxima>th_cor
        else
            cornerness(ii,jj)=0;
        end
    end
end
% extract feature by comparing cornerness values
corsize=size(cornerness);
corarray=reshape(cornerness',1,corsize(1)*corsize(2));
[corsort,index_corarray]=sort(corarray);
index=index_corarray(end:-1:end-topnum+1)-1;
posx=floor(index/corsize(2))+1;
posy=index-(posx-1)*corsize(2)+1;
featurematrix=zeros(corsize(1),corsize(2));
featurematrix(posx,posy)=1;

function [gx,gy]=getGradient(im,x,y)

x=x+1;
y=y+1;

% % use sobel filter
% patch=im(x-1:x+1,y-1:y+1);
% gx=[-1,0,1]*patch*[1;2;1]/8;% finite difference for x
% gy=[1,2,1]*patch*[-1;0;1]/8;

% use finite difference
gx=(im(x+1,y)-im(x-1,y))/2;% finite difference for x
gy=(im(x,y+1)-im(x,y-1))/2;


