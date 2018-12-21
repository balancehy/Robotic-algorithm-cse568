% Each row of winmatrix represents the template setup for each image. the
% first two elements in each row are the postions of template center, and
% the third element is half of window size. 320,30,20;
winmatrix=[150,150,40;
           140,125,20;
           320,30,20;
           250,220,30;
           170,220,30;
           170,220,30];


col2chop=100:300;
% index=find(all(image_orig(:,col2chop)<50,2)==1);
separcell={[1,348;349,689;690,1024],[1,348;349,685;686,1024],[1,340;341,680;681,1024],...
    [1,338;339,675;676,1024],[1,344;345,685;686,1024],[1,335;336,674;675,1024]};% lines at which multipe imgages should be separated

for ii=1:6
    clear Rim Gim Bim Rim_shift Gim_shift imcolor
    win=winmatrix(ii,:);
    num=ii;
    
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

    [Gim_shift,Rim_shift,tmpl]= align1(Rim,Gim,Bim,win);
    figure((ii-1)*3+3)
    imshow(tmpl)
    imwrite(tmpl,strcat(dirstring,"/template",num2str(num),'-ssd',".jpg"))
    imcolor(:,:,1)=Rim_shift;
    imcolor(:,:,2)=Gim_shift;
    imcolor(:,:,3)=Bim;
    figure((ii-1)*3+2)
    image(imcolor)
    imwrite(imcolor,strcat(dirstring,"/image",num2str(num),'-ssd',".jpg")) % aligned image
    
    imcolor(:,:,1)=Rim;
    imcolor(:,:,2)=Gim;
    imcolor(:,:,3)=Bim; 
    figure((ii-1)*3+1)
    image(imcolor)
    imwrite(imcolor,strcat(dirstring,"/image",num2str(num),'-color',".jpg"))

%     testim=[Rim_shift,Gim_shift,Bim];
%     imwrite(testim,"test.jpg")
    % testim=ones(200,200)*0;
%     imwrite(testim,strcat("imageseparate/","sepa",num2str(num),".jpg"))
end

% using SSD
function [Gim_shift,Rim_shift,tmpl]=align1(Rim,Gim,Bim,win)
u=win(1);% template window x postion
v=win(2);% template window y postion
m=win(3);% template window size. 2m+1-by-2m+1
tmpl=Bim(u-m:u+m,v-m:v+m);% extract template from B channel

m_simi=getSimimatrix(Gim,tmpl,m);
[Vmin,Irow,Icol]=minssd(m_simi);
du=Irow+m-u;% row difference from target image to fixed image(B channel)
dv=Icol+m-v;% column difference
Gim_shift=circshift(Gim,[-du,-dv]);
forprint=[-du,-dv];

m_simi=getSimimatrix(Rim,tmpl,m);
[Vmin,Irow,Icol]=minssd(m_simi);
du=Irow+m-u;% row difference from target image to fixed image(B channel). +m transfers to target matrix domain
dv=Icol+m-v;% column difference
Rim_shift=circshift(Rim,[-du,-dv]);
forprint=[-du,-dv];
% sprintf('The shift from G to B is: [%d,%d]\nThe shift from R to B is: [%d,%d]',forprint(1,1),forprint(1,2),forprint(2,1),forprint(2,2))
end


function m_simi=getSimimatrix(T,tmpl,m)
[row,col]=size(T);% (m+1,m+1),(row-m,m+1),(m+1,col-m),(row-m,col+m) are the boundaries of searching region in target image
m_simi=ones(row-2*m,col-2*m);% initialize similarity score matrix
for ii=m+1:row-m% ii,jj are index in targe matrix
    for jj=m+1:col-m
        m_simi(ii-m,jj-m)=ssd(tmpl,T(ii-m:ii+m,jj-m:jj+m));
    end
end
end
function y=ssd(m1,m2)
if isequal(size(m1),size(m2))
    y=sum(sum((double(m1)-double(m2)).^2));
else
    sprint("two matrix size are not matched")
end
end

function [Vmin,I_row,I_col]=minssd(m_simi)
[Vrow,Irow]=min(m_simi);
[Vmin,Icol]=min(Vrow);
I_row=Irow(Icol);
I_col=Icol;
end