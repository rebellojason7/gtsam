/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTreeCliqueBase-inl.h
 * @brief   Base class for cliques of a BayesTree
 * @author  Richard Roberts and Frank Dellaert
 */

#pragma once

#include <gtsam/inference/GenericSequentialSolver.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED, CONDITIONAL>::assertInvariants() const {
#ifndef NDEBUG
    // We rely on the keys being sorted
//    FastVector<Index> sortedUniqueKeys(conditional_->begin(), conditional_->end());
//    std::sort(sortedUniqueKeys.begin(), sortedUniqueKeys.end());
//    std::unique(sortedUniqueKeys.begin(), sortedUniqueKeys.end());
//    assert(sortedUniqueKeys.size() == conditional_->size() &&
//        std::equal(sortedUniqueKeys.begin(), sortedUniqueKeys.end(), conditional_->begin()));
#endif
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  std::vector<Index> BayesTreeCliqueBase<DERIVED, CONDITIONAL>::separator_setminus_B(
      derived_ptr B) const {
    sharedConditional p_F_S = this->conditional();
    std::vector<Index> &indicesB = B->conditional()->keys();
    std::vector<Index> S_setminus_B;
    std::set_difference(p_F_S->beginParents(), p_F_S->endParents(), //
        indicesB.begin(), indicesB.end(), back_inserter(S_setminus_B));
    return S_setminus_B;
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  std::vector<Index> BayesTreeCliqueBase<DERIVED, CONDITIONAL>::shortcut_indices(
      derived_ptr B, const FactorGraph<FactorType>& p_Cp_B) const {
    std::set<Index> allKeys = p_Cp_B.keys();
    std::vector<Index> &indicesB = B->conditional()->keys();
    std::vector<Index> S_setminus_B = separator_setminus_B(B); // TODO, get as argument?
    std::vector<Index> keep;
    std::set_intersection(S_setminus_B.begin(), S_setminus_B.end(), //
        allKeys.begin(), allKeys.end(), back_inserter(keep));
    std::set_intersection(indicesB.begin(), indicesB.end(), //
        allKeys.begin(), allKeys.end(), back_inserter(keep));
    // BOOST_FOREACH(Index j, keep) std::cout << j << " "; std::cout << std::endl;
    return keep;
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  BayesTreeCliqueBase<DERIVED, CONDITIONAL>::BayesTreeCliqueBase(
      const sharedConditional& conditional) :
      conditional_(conditional) {
    assertInvariants();
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  BayesTreeCliqueBase<DERIVED, CONDITIONAL>::BayesTreeCliqueBase(
      const std::pair<sharedConditional,
          boost::shared_ptr<typename ConditionalType::FactorType> >& result) :
      conditional_(result.first) {
    assertInvariants();
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED, CONDITIONAL>::print(const std::string& s,
      const IndexFormatter& indexFormatter) const {
    conditional_->print(s, indexFormatter);
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  size_t BayesTreeCliqueBase<DERIVED, CONDITIONAL>::treeSize() const {
    size_t size = 1;
    BOOST_FOREACH(const derived_ptr& child, children_)
      size += child->treeSize();
    return size;
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED, CONDITIONAL>::printTree(
      const std::string& indent, const IndexFormatter& indexFormatter) const {
    asDerived(this)->print(indent, indexFormatter);
    BOOST_FOREACH(const derived_ptr& child, children_)
      child->printTree(indent + "  ", indexFormatter);
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED, CONDITIONAL>::permuteWithInverse(
      const Permutation& inversePermutation) {
    conditional_->permuteWithInverse(inversePermutation);
    BOOST_FOREACH(const derived_ptr& child, children_) {
      child->permuteWithInverse(inversePermutation);
    }
    assertInvariants();
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  bool BayesTreeCliqueBase<DERIVED, CONDITIONAL>::permuteSeparatorWithInverse(
      const Permutation& inversePermutation) {
    bool changed = conditional_->permuteSeparatorWithInverse(
        inversePermutation);
#ifndef NDEBUG
    if(!changed) {
      BOOST_FOREACH(Index& separatorKey, conditional_->parents()) {assert(separatorKey == inversePermutation[separatorKey]);}
      BOOST_FOREACH(const derived_ptr& child, children_) {
        assert(child->permuteSeparatorWithInverse(inversePermutation) == false);
      }
    }
#endif
    if (changed) {
      BOOST_FOREACH(const derived_ptr& child, children_) {
        (void) child->permuteSeparatorWithInverse(inversePermutation);
      }
    }
    assertInvariants();
    return changed;
  }

  /* ************************************************************************* */
  // The shortcut density is a conditional P(S|R) of the separator of this
  // clique on the root. We can compute it recursively from the parent shortcut
  // P(Sp|R) as \int P(Fp|Sp) P(Sp|R), where Fp are the frontal nodes in p
  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  BayesNet<CONDITIONAL> BayesTreeCliqueBase<DERIVED, CONDITIONAL>::shortcut(
      derived_ptr B, Eliminate function) const {

    // Check if the ShortCut already exists
    if (!cachedShortcut_) {

      // We only calculate the shortcut when this clique is not B
      // and when the S\B is not empty
      std::vector<Index> S_setminus_B = separator_setminus_B(B);
      if (B.get() != this && !S_setminus_B.empty()) {

        // Obtain P(Cp||B) = P(Fp|Sp) * P(Sp||B) as a factor graph
        derived_ptr parent(parent_.lock());
        FactorGraph<FactorType> p_Cp_B(parent->shortcut(B, function)); // P(Sp||B)
        p_Cp_B.push_back(parent->conditional()->toFactor()); // P(Fp|Sp)

        // Add the root conditional
        // TODO: this is needed because otherwise we will be solving singular
        // systems and exceptions are thrown. However, we should be able to omit
        // this if we can get ATTEMPT_AT_NOT_ELIMINATING_ALL in
        // GenericSequentialSolver.* working...
        p_Cp_B.push_back(B->conditional()->toFactor()); // P(B)

        // Create solver that will marginalize for us
        GenericSequentialSolver<FactorType> solver(p_Cp_B);

        // Determine the variables we want to keep
        std::vector<Index> keep = shortcut_indices(B, p_Cp_B);

        // Finally, we only want to have S\B variables in the Bayes net, so
        size_t nrFrontals = S_setminus_B.size();
        cachedShortcut_ = //
            *solver.conditionalBayesNet(keep, nrFrontals, function);
        assertInvariants();
      } else {
        BayesNet<CONDITIONAL> empty;
        cachedShortcut_ = empty;
      }
    }

    // return the shortcut P(S||B)
    return *cachedShortcut_; // return the cached version
  }

  /* ************************************************************************* */
  // P(C) = \int_R P(F|S) P(S|R) P(R)
  // TODO: Maybe we should integrate given parent marginal P(Cp),
  // \int(Cp\S) P(F|S)P(S|Cp)P(Cp)
  // Because the root clique could be very big.
  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  FactorGraph<typename BayesTreeCliqueBase<DERIVED, CONDITIONAL>::FactorType> BayesTreeCliqueBase<
      DERIVED, CONDITIONAL>::marginal(derived_ptr R, Eliminate function) const {
    // If we are the root, just return this root
    // NOTE: immediately cast to a factor graph
    BayesNet<ConditionalType> bn(R->conditional());
    if (R.get() == this)
      return bn;

    // Combine P(F|S), P(S|R), and P(R)
    BayesNet<ConditionalType> p_FSR = this->shortcut(R, function);
    p_FSR.push_front(this->conditional());
    p_FSR.push_back(R->conditional());

    assertInvariants();
    GenericSequentialSolver<FactorType> solver(p_FSR);
    return *solver.jointFactorGraph(conditional_->keys(), function);
  }

  /* ************************************************************************* */
  // separator marginal, uses separator marginal of parent recursively
  // P(C) = P(F|S) P(S)
  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  FactorGraph<typename BayesTreeCliqueBase<DERIVED, CONDITIONAL>::FactorType> BayesTreeCliqueBase<
      DERIVED, CONDITIONAL>::separatorMarginal(derived_ptr R, Eliminate function) const {
    // Check if the Separator marginal was already calculated
    if (!cachedSeparatorMarginal_) {

      // If this is the root, there is no separator
      if (R.get() == this) {
        // we are root, return empty
        FactorGraph<FactorType> empty;
        cachedSeparatorMarginal_ = empty;
      } else {
        // Obtain P(S) = \int P(Cp) = \int P(Fp|Sp) P(Sp)
        // initialize P(Cp) with the parent separator marginal
        derived_ptr parent(parent_.lock());
        FactorGraph<FactorType> p_Cp(parent->separatorMarginal(R, function)); // P(Sp)
        // now add the parent cobnditional
        p_Cp.push_back(parent->conditional()->toFactor()); // P(Fp|Sp)

        // Create solver that will marginalize for us
        GenericSequentialSolver<FactorType> solver(p_Cp);

        // The variables we want to keep are exactly the ones in S
        sharedConditional p_F_S = this->conditional();
        std::vector<Index> indicesS(p_F_S->beginParents(), p_F_S->endParents());

        cachedSeparatorMarginal_ = *(solver.jointBayesNet(indicesS, function));
      }
    }

    // return the shortcut P(S||B)
    return *cachedSeparatorMarginal_; // return the cached version
  }

  /* ************************************************************************* */
  // marginal2, uses separator marginal of parent recursively
  // P(C) = P(F|S) P(S)
  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  FactorGraph<typename BayesTreeCliqueBase<DERIVED, CONDITIONAL>::FactorType> BayesTreeCliqueBase<
      DERIVED, CONDITIONAL>::marginal2(derived_ptr R, Eliminate function) const {
    // initialize with separator marginal P(S)
    FactorGraph<FactorType> p_C(this->separatorMarginal(R, function));
    // add the conditional P(F|S)
    p_C.push_back(this->conditional()->toFactor());
    return p_C;
  }

  /* ************************************************************************* */
  // P(C1,C2) = \int_R P(F1|S1) P(S1|R) P(F2|S1) P(S2|R) P(R)
  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  FactorGraph<typename BayesTreeCliqueBase<DERIVED, CONDITIONAL>::FactorType> BayesTreeCliqueBase<
      DERIVED, CONDITIONAL>::joint(derived_ptr C2, derived_ptr R,
      Eliminate function) const {
    // For now, assume neither is the root

    sharedConditional p_F1_S1 = this->conditional();
    sharedConditional p_F2_S2 = C2->conditional();

    // Combine P(F1|S1), P(S1|R), P(F2|S2), P(S2|R), and P(R)
    FactorGraph<FactorType> joint;
    if (!isRoot()) {
      joint.push_back(p_F1_S1->toFactor()); // P(F1|S1)
      joint.push_back(shortcut(R, function)); // P(S1|R)
    }
    if (!C2->isRoot()) {
      joint.push_back(p_F2_S2->toFactor()); // P(F2|S2)
      joint.push_back(C2->shortcut(R, function)); // P(S2|R)
    }
    joint.push_back(R->conditional()->toFactor()); // P(R)

    // Merge the keys of C1 and C2
    std::vector<Index> keys12;
    std::vector<Index> &indices1 = p_F1_S1->keys(), &indices2 = p_F2_S2->keys();
    std::set_union(indices1.begin(), indices1.end(), //
        indices2.begin(), indices2.end(), std::back_inserter(keys12));

    // Check validity
    bool cliques_intersect = (keys12.size() < indices1.size() + indices2.size());
    if (!isRoot() && !C2->isRoot() && cliques_intersect)
      throw std::runtime_error(
          "BayesTreeCliqueBase::joint can only calculate joint if cliques are disjoint\n"
          "or one of them is the root clique");

    // Calculate the marginal
    assertInvariants();
    GenericSequentialSolver<FactorType> solver(joint);
    return *solver.jointFactorGraph(keys12, function);
  }

  /* ************************************************************************* */
  template<class DERIVED, class CONDITIONAL>
  void BayesTreeCliqueBase<DERIVED, CONDITIONAL>::deleteCachedShorcuts() {

    // When a shortcut is requested, all of the shortcuts between it and the
    // root are also generated. So, if this clique's cached shortcut is set,
    // recursively call over all child cliques. Otherwise, it is unnecessary.
    if (cachedShortcut_) {
      BOOST_FOREACH(derived_ptr& child, children_) {
        child->deleteCachedShorcuts();
      }

      //Delete CachedShortcut for this clique
      this->resetCachedShortcut();
    }

  }

}
