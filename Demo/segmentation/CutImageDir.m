function CutImageDir(dir_name)
    mask_dir = [dir_name '/mask/'];
    result_dir = [dir_name '/result/'];
    mkdir(mask_dir);
    mkdir(result_dir);
    %sn = [13293115, 13293116, 13502663, 13502667, 13502670, 13502673, 13502680, 13502683, 13502686, 13502693];
%    isrotated = [1 0 1 1 1 1 0 0 1 0];
    isrotated = ones(10,1);
    for j=1:10
        bk_img = imread(sprintf('%s//25_Cam%d.jpg', dir_name, j-1));
        if isrotated(j)
            bk_img = rot90(bk_img,1);
            imwrite(bk_img, sprintf('%s//25_Cam%d.jpg', dir_name, j-1));
        end
        scale = 800/size(bk_img,1);
        bk_img = imresize(bk_img,scale);
        bk_img = im2double(bk_img);
        bk_img = imfilter(bk_img,fspecial('disk',2));
        mean_b = mean(bk_img,3)-eps;
        bk_img2 = bsxfun(@minus, bk_img, mean_b);
        parfor i=1:24
            CutOneImage(sprintf('%s//%.2d_Cam%d.jpg', dir_name, i, j-1), ...
               bk_img, bk_img2, mean_b, sprintf('%s//%.2d_Cam%d.jpg', mask_dir, i, j-1), ...
               sprintf('%s//%.2d_Cam%d.jpg', result_dir, i, j-1), isrotated(j)); 
        end
    end
end

function CutOneImage(img1_name,img2,img2_,mean_img2,mask_name,temp_name,isrotated)
    img1 = imread(img1_name);
    if isrotated
        img1 = rot90(img1,1);
        imwrite(img1, img1_name, 'Quality', 80);
    end
    scale = 800/size(img1,1);
    img1 = imresize(img1,scale);
    img1 = im2double(img1);
    [h,w,~] = size(img1);
    img1 = imfilter(img1,fspecial('disk',2));
    delta=sum(abs(img1-img2),3); 
    a = bsxfun(@minus, img1, mean_img2);
    S = sum(a.*img2_, 3)./sqrt(sum(a.*a,3).*sum(img2_.*img2_,3));
    mask = S < 0.4;
    mask = imclose(mask,strel('disk',4));
    mask = logical(imfill(double(mask)));
    mask = imopen(mask,strel('disk',10));
    [L,~] = bwlabeln(mask,4);
    mask = zeros(h,w);
    mask(L==L(h/2,w/2)) = 1;
    img1 = rgb2gray(img1);
    img0 = img1;
    img0(~mask) = 0;
    mask = 1 - RegionGrowing(img0, 0.2, [1,1]);
    mask(delta < 0.15) = 0;
    mask = imclose(logical(mask),strel('disk',4));
    mask = logical(imfill(double(mask)));
    mask = imopen(mask,strel('disk',10));
    [I,J] = ind2sub(size((mask)),find(mask>0));
    x = round(sum(I)/length(I));
    y = round(sum(J)/length(J));
    [L,~]=bwlabeln(mask,4);
    mask = zeros(h,w);
    mask(L==L(x,y)) = 1;
    mask = logical(mask);
    img1(~mask) = 0;
    imwrite(img1, temp_name);
    mask = imresize(mask,1/scale);
    mask = logical(mask);
    imwrite(mask, mask_name);
end