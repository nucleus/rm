#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <vector>
#include <iostream>

#include <util/interface/BoundingBox.hpp>

namespace util {
	template <class T>
	class KDTreeNode {
	public:
		typedef std::vector<T*> ObjectVector;

	public:
		KDTreeNode(Axis _axis = X, float _split = 0.0f): 
			m_axis(_axis), m_split(_split), m_objects(NULL), m_children(NULL) {

			m_objects = new ObjectVector();
		}
		
	public:
		ObjectVector& objects() { return *m_objects; }

		KDTreeNode<T>* leftChild() { return m_children; }
		KDTreeNode<T>* rightChild() { return m_children + 1; }

	public:
		void setLeaf() {
			m_children = NULL;
		}

		bool leaf() const {
			return m_children == NULL;
		}
		
		void allocateChildren() {
			m_children = new KDTreeNode<T>[2];
		}

		void clear() {
			if (m_children) {
				leftChild()->clear();
				rightChild()->clear();
				delete [] m_children;
				m_children = NULL;
			}

			if (m_objects) {
				delete m_objects;
				m_objects = NULL;
			}
		}
		
	public:
		Axis m_axis;
		float m_split;

	protected:
		ObjectVector* m_objects;
		KDTreeNode<T>* m_children;
	};
	
	template <class T>
	class KDTree {
	public:
		KDTree(): m_root(NULL) {}
		KDTree(std::vector<T*>& objs, const util::BoundingBox& box) {
			build(objs, box);
		}
		
	public:
		void clear() {
			if (!m_root) { return; }
			m_root->clear();
			delete m_root;
			m_root = NULL;
		}
		
		void build(std::vector<T*>& objs, const util::BoundingBox& box) {
			if (m_root) { return; }
			m_root = new KDTreeNode<T>();
			m_root->objects().insert(m_root->objects().begin(), objs.begin(), objs.end());
			subdivide(m_root, box);
		}
		
		void print(std::ostream& out) const {
			printNode(out, m_root);
		}
		
	protected:
		virtual void subdivide(KDTreeNode<T>* node, const util::BoundingBox& box, unsigned depth = 0) = 0;

		virtual void printNode(std::ostream& out, KDTreeNode<T>* node, unsigned indents = 0) const = 0;
		
	protected:
		KDTreeNode<T>* m_root;
	};
}

#endif
