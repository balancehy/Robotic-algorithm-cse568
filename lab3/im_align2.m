% template setup
winmatrix=[150,150,10;
           200,155,10;
           140,210,10;
           250,220,15;
           170,220,10;
           170,220,15];
for ii=1:6
    clear Rim Gim Bim Rim_shift Gim_shift imcolor
    win=winmatrix(ii,:);
    num=ii;
    imagename=strcat("image",num2str(num),".jpg");
    image_orig=imread(imagename);
    col=length(image_orig(1,:));
    
    col2chop=100:300;
    index=find(all(image_orig(:,col2chop)<50,2)==1);
    separcell={[1,348;349,689;690,1024],[1,348;349,685;686,1024],[1,340;341,680;681,1024],...
        [1,338;339,675;676,1024],[1,344;345,685;686,1024],[1,335;336,674;675,1024]};% lines at which multipe imgages shoudl be separated
    
    separator=separcell{num};
    Bsize=separator(1,2)-separator(1,1);% top
    Gsize=separator(2,2)-separator(2,1);% middile
    Rsize=separator(3,2)-separator(3,1);% bottom
    
    sizediff=abs([Bsize,Gsize,Rsize]-max([Bsize,Gsize,Rsize]));
    Bim=[image_orig(separator(1,1):separator(1,2),:);ones(sizediff(1),col)*255];% top B
    Gim=[image_orig(separator(2,1):separator(2,2),:);ones(sizediff(2),col)*255];% middle G
    Rim=[image_orig(separator(3,1):separator(3,2),:);ones(sizediff(3),col)*255];% bottom R
    
    
    [Gim_shift,Rim_shift,tmpl]= align2(Rim,Gim,Bim,win);
    figure((ii-1)*3+3)
    imshow(tmpl)
    imwrite(tmpl,strcat(dirstring,"/template",num2str(num),'-ncc',".jpg"))
    imcolor(:,:,1)=Rim_shift;
    imcolor(:,:,2)=Gim_shift;
    imcolor(:,:,3)=Bim;
    figure((ii-1)*3+2)
    image(imcolor)
    imwrite(imcolor,strcat(dirstring,"/image",num2str(num),'-ncc',".jpg")) % aligned image
    
    imcolor(:,:,1)=Rim;
    imcolor(:,:,2)=Gim;
    imcolor(:,:,3)=Bim; 
    figure((ii-1)*3+1)
    image(imcolor)
%     imwrite(imcolor,strcat("imagealign2/Notaligned_",num2str(num),".jpg"))



%     testim=[Rim_shift,Gim_shift,Bim];
%     imwrite(testim,"test.jpg")
    % testim=ones(200,200)*0;
%     imwrite(testim,strcat("imageseparate/","sepa",num2str(num),".jpg"))
end

% using NCC
function [Gim_shift,Rim_shift,tmpl]= align2(Rim,Gim,Bim,win)
u=win(1);% template window x postion
v=win(2);% template window y postion
m=win(3);% template window size. 2m+1-by-2m+1
tmpl=Bim(u-m:u+m,v-m:v+m);% extract template from B channel

m_simi=getSimimatrix(Gim,tmpl,m);
[Vmin,Irow,Icol]=maxncc(m_simi);% m_simi is smaller than orignal image by m rows and m columns
du=Irow+m-u;% row difference in original image from target image to fixed image(B channel)
dv=Icol+m-v;% column difference
Gim_shift=circshift(Gim,[-du,-dv]);
forprint=[-du,-dv];

m_simi=getSimimatrix(Rim,tmpl,m);
[Vmin,Irow,Icol]=maxncc(m_simi);
du=Irow+m-u;% row difference from target image to fixed image(B channel). +m transfers to target matrix domain
dv=Icol+m-v;% column difference
Rim_shift=circshift(Rim,[-du,-dv]);
forprint=[forprint;-du,-dv];

% sprintf('The shift from G to B is: [%d,%d]\nThe shift from R to B is: [%d,%d]',forprint(1,1),forprint(1,2),forprint(2,1),forprint(2,2))
end


function m_simi=getSimimatrix(T,tmpl,m)
[row,col]=size(T);% (m+1,m+1),(row-m,m+1),(m+1,col-m),(row-m,col+m) are the boundaries of searching region in target image
m_simi=ones(row-2*m,col-2*m);% initialize similarity score matrix
for ii=m+1:row-m% ii,jj are index in targe matrix
    for jj=m+1:col-m
        m_simi(ii-m,jj-m)=ncc(tmpl,T(ii-m:ii+m,jj-m:jj+m));% use normalized cross correlation
    end
end
end

function y=ncc(m1,m2)
size1=size(m1);
size2=size(m2);
if isequal(size1,size2)
    m1=double(m1);
    m2=double(m2);
    mean1=sum(sum(m1))/(size1(1)*size1(2));
    mean2=sum(sum(m2))/(size2(1)*size2(2));
    y=sum(sum((m1-mean1).*(m2-mean2)))/sqrt(sum(sum((m1-mean1).^2))*sum(sum((m2-mean2).^2)));
%     y=sum(sum((double(m1)-double(m2)).^2));
else
    sprint("two matrix size are not matched")
end
end

function [Vmin,I_row,I_col]=maxncc(m_simi)
[Vrow,Irow]=max(m_simi);
[Vmin,Icol]=max(Vrow);
I_row=Irow(Icol);
I_col=Icol;
end