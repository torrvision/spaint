set makeprg=[[\ -f\ Makefile\ ]]\ &&\ make\ \\\|\\\|\ make\ -C\ build/

" recreate tags file with F5
map <F5> :!ctags -R --sort=yes --c++-kinds=+p --fields=+iaS --extra=+q --exclude=libraries -f tags .<CR>
map <S-F5> :!ctags -R --sort=yes --c++-kinds=+p --fields=+iaS --extra=+q -L libraries/ -f librarytags .<CR>
set tags+=librarytags

